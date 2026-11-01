"""
Training Pipeline for Degradation-Aware Neural Network (DA-Net).
Handles device management (Apple Silicon MPS / CUDA / CPU), checkpointing,
metrics tracking, and validation on personal recordings.
"""

import os
import time
from typing import Dict, Any, Optional
import numpy as np
import torch
from torch.utils.data import DataLoader
from .model import DegradationCovarianceNet
from .loss import MultiTaskDegradationLoss
from .dataset import RoverPersonalDataset


class RoverTrainer:
    """End-to-end training engine for personal rover telemetry data."""

    def __init__(self,
                 model: DegradationCovarianceNet,
                 train_dataset: RoverPersonalDataset,
                 val_dataset: Optional[RoverPersonalDataset] = None,
                 batch_size: int = 32,
                 lr: float = 1e-3,
                 weight_decay: float = 1e-4,
                 device: Optional[str] = None,
                 checkpoint_dir: str = 'checkpoints'):

        # Automatic device selection (supports Apple Silicon MPS, CUDA, CPU)
        if device is None:
            if torch.backends.mps.is_available():
                self.device = torch.device('mps')
            elif torch.cuda.is_available():
                self.device = torch.device('cuda')
            else:
                self.device = torch.device('cpu')
        else:
            self.device = torch.device(device)

        self.model = model.to(self.device)
        self.train_dataset = train_dataset
        self.val_dataset = val_dataset
        self.batch_size = batch_size
        self.checkpoint_dir = checkpoint_dir
        os.makedirs(checkpoint_dir, exist_ok=True)

        self.train_loader = DataLoader(
            train_dataset,
            batch_size=batch_size,
            shuffle=True,
            drop_last=len(train_dataset) > batch_size
        )

        self.val_loader = DataLoader(
            val_dataset,
            batch_size=batch_size,
            shuffle=False
        ) if val_dataset is not None else None

        self.criterion = MultiTaskDegradationLoss().to(self.device)
        self.optimizer = torch.optim.AdamW(
            self.model.parameters(),
            lr=lr,
            weight_decay=weight_decay
        )
        self.scheduler = torch.optim.lr_scheduler.CosineAnnealingLR(
            self.optimizer,
            T_max=50,
            eta_min=1e-5
        )

        self.history = {'train_loss': [], 'val_loss': [], 'val_rmse': []}

    def train_epoch(self) -> float:
        self.model.train()
        total_loss = 0.0
        n_batches = 0

        for feats, gt_vel in self.train_loader:
            feats = feats.to(self.device)
            gt_vel = gt_vel.to(self.device)

            self.optimizer.zero_grad()
            weights, cov_scale, v_pred, v_logvar = self.model(feats)

            losses = self.criterion(v_pred, v_logvar, gt_vel, weights, feats)
            losses['total'].backward()

            # Gradient clipping for numerical stability
            torch.nn.utils.clip_grad_norm_(self.model.parameters(), max_norm=2.0)
            self.optimizer.step()

            total_loss += losses['total'].item()
            n_batches += 1

        self.scheduler.step()
        return total_loss / max(1, n_batches)

    def evaluate(self) -> Dict[str, float]:
        if self.val_loader is None:
            return {}

        self.model.eval()
        total_loss = 0.0
        sq_errors = []
        n_batches = 0

        with torch.no_grad():
            for feats, gt_vel in self.val_loader:
                feats = feats.to(self.device)
                gt_vel = gt_vel.to(self.device)

                weights, cov_scale, v_pred, v_logvar = self.model(feats)
                losses = self.criterion(v_pred, v_logvar, gt_vel, weights, feats)

                total_loss += losses['total'].item()
                diff = (v_pred - gt_vel).cpu().numpy()
                sq_errors.append(diff ** 2)
                n_batches += 1

        val_loss = total_loss / max(1, n_batches)
        rmse = float(np.sqrt(np.mean(np.concatenate(sq_errors, axis=0))))
        return {'val_loss': val_loss, 'val_rmse': rmse}

    def fit(self, epochs: int = 40, patience: int = 10) -> Dict[str, Any]:
        """Run full training routine."""
        print(f"🚀 Training DA-Net on device: {self.device}")
        print(f"   Train samples: {len(self.train_dataset)} | Batch size: {self.batch_size}")
        if self.val_dataset:
            print(f"   Val samples: {len(self.val_dataset)}")

        best_val_loss = float('inf')
        no_improve_epochs = 0

        start_time = time.time()
        for epoch in range(1, epochs + 1):
            train_loss = self.train_epoch()
            self.history['train_loss'].append(train_loss)

            msg = f"Epoch [{epoch:03d}/{epochs:03d}] Train Loss: {train_loss:.4f}"

            if self.val_loader:
                val_metrics = self.evaluate()
                val_loss = val_metrics['val_loss']
                val_rmse = val_metrics['val_rmse']
                self.history['val_loss'].append(val_loss)
                self.history['val_rmse'].append(val_rmse)

                msg += f" | Val Loss: {val_loss:.4f} | Val RMSE: {val_rmse:.4f} m/s"

                # Checkpoint best model
                if val_loss < best_val_loss:
                    best_val_loss = val_loss
                    no_improve_epochs = 0
                    self.save_checkpoint('best_model.pt')
                    msg += " ★ (Best)"
                else:
                    no_improve_epochs += 1
            else:
                self.save_checkpoint('last_model.pt')

            print(msg)

            if no_improve_epochs >= patience:
                print(f"⏹️ Early stopping triggered after {patience} epochs with no improvement.")
                break

        total_time = time.time() - start_time
        print(f"✅ Training completed in {total_time:.2f}s.")

        # Save normalization parameters next to checkpoint
        norm_path = os.path.join(self.checkpoint_dir, 'norm_params.json')
        self.train_dataset.export_norm_params(norm_path)
        print(f"📁 Exported normalization statistics to: {norm_path}")

        return self.history

    def save_checkpoint(self, filename: str):
        path = os.path.join(self.checkpoint_dir, filename)
        torch.save({
            'model_state_dict': self.model.state_dict(),
            'optimizer_state_dict': self.optimizer.state_dict(),
            'in_features': self.model.in_features,
            'hidden_dim': self.model.hidden_dim,
        }, path)

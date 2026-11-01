"""
Multi-Task Loss Functions for Degradation-Aware State Estimation.
Implements:
1. Heteroscedastic Negative Log-Likelihood (NLL) with learned aleatoric uncertainty.
2. Degradation-Physics Consistency Regularization.
3. Smoothness constraints on dynamic covariance scaling.
"""

from typing import Dict
import torch
import torch.nn as nn
import torch.nn.functional as F


class MultiTaskDegradationLoss(nn.Module):
    """
    Combined loss for training on personal vehicle telemetry recordings.
    """
    def __init__(self,
                 lambda_nll: float = 1.0,
                 lambda_huber: float = 0.5,
                 lambda_reg: float = 0.1):
        super().__init__()
        self.lambda_nll = lambda_nll
        self.lambda_huber = lambda_huber
        self.lambda_reg = lambda_reg
        self.huber = nn.HuberLoss(delta=0.5)

    def forward(self,
                v_pred: torch.Tensor,
                v_logvar: torch.Tensor,
                v_gt: torch.Tensor,
                weights: torch.Tensor,
                raw_features: torch.Tensor) -> Dict[str, torch.Tensor]:
        """
        Args:
            v_pred: (Batch, 3) predicted body velocity
            v_logvar: (Batch, 3) predicted log(sigma^2)
            v_gt: (Batch, 3) ground truth velocity
            weights: (Batch, 4) [radar, lidar, cam, wheel]
            raw_features: (Batch, Seq_Len, 16) or (Batch, 16)
        """
        # 1. Heteroscedastic Gaussian Negative Log-Likelihood
        # 0.5 * (exp(-s) * (pred - gt)^2 + s)
        precision = torch.exp(-v_logvar)
        sq_diff = (v_pred - v_gt) ** 2
        loss_nll = 0.5 * torch.mean(precision * sq_diff + v_logvar)

        # 2. Huber Loss directly on velocity
        loss_huber = self.huber(v_pred, v_gt)

        # 3. Physics Degradation Regularization
        # If LiDAR backscatter (index 8) is high, LiDAR weight should be penalized if it stays high
        if raw_features.dim() == 3:
            backscatter = raw_features[:, -1, 8]  # last step in sequence
        else:
            backscatter = raw_features[:, 8]

        w_lidar = weights[:, 1]
        loss_reg = torch.mean(F.relu(w_lidar * backscatter - 0.2))

        # Total Loss
        total_loss = (self.lambda_nll * loss_nll +
                      self.lambda_huber * loss_huber +
                      self.lambda_reg * loss_reg)

        return {
            'total': total_loss,
            'nll': loss_nll,
            'huber': loss_huber,
            'reg': loss_reg
        }

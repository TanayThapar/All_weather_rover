"""
All Weather Rover: Degradation-Aware 4D Radar-Inertial-Visual State Estimation.
"""

from .core.state_vector import RoverState
from .core.doppler_kinematics import RadarTarget, RadarDopplerKinematics
from .core.ekf_fusion import DegradationAwareEKF
from .learning.model import DegradationCovarianceNet, NumPyDegradationPredictor
from .learning.dataset import RoverPersonalDataset

__version__ = "0.1.0"
__author__ = "Tanay Thapar"

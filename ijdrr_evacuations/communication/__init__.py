"""Vehicular-DTN messages, contact detection, and forwarding."""

from .buffer import BufferInsertResult, MessageBuffer
from .contacts import (
    Contact,
    ContactDetector,
    ContactHistory,
    ContactUpdate,
    SpatialHashContactDetector,
)
from .forwarding import EpidemicPolicy, ForwardingPolicy
from .manager import CommunicationManager, CommunicationTickResult, TransferRecord
from .message import Message, MessageHeader, MessageKind, MessagePayloadValue
from .registry import MessageRegistry

__all__ = [
    "BufferInsertResult",
    "CommunicationManager",
    "CommunicationTickResult",
    "Contact",
    "ContactDetector",
    "ContactHistory",
    "ContactUpdate",
    "EpidemicPolicy",
    "ForwardingPolicy",
    "Message",
    "MessageBuffer",
    "MessageHeader",
    "MessageKind",
    "MessagePayloadValue",
    "MessageRegistry",
    "SpatialHashContactDetector",
    "TransferRecord",
]

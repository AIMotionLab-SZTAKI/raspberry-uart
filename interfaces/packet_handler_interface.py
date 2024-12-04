from abc import abstractmethod, ABCMeta

"""
Interface for packet handlers
Every service type packet handler should implement this class
"""
class PacketHandlerInterface(metaclass = ABCMeta):

    def __init_subclass__(subclass, **kwargs):
        if subclass.packet_decomposition == PacketHandlerInterface.packet_decomposition:
            raise NotImplementedError(
               'Subclasses of `PacketHandlerInterface` must override the `packet_decomposition` method'
            )
        if subclass.packet_composition == PacketHandlerInterface.packet_composition:
            raise NotImplementedError(
               'Subclasses of `PacketHandlerInterface` must override the `packet_composition` method'
            )
        super().__init_subclass__(**kwargs)


    @staticmethod
    @abstractmethod
    def packet_decomposition(*args, **kwargs):
        raise NotImplementedError
    
    
    @staticmethod
    @abstractmethod
    def packet_composition(*args, **kwargs):
        raise NotImplementedError
    


class FormationNote():
    @staticmethod
    def getName():
        raise NotImplementedError

class SquareNote(FormationNote):
    @staticmethod
    def getName():
        return "Square"

class LinearNote(FormationNote):
    @staticmethod
    def getName():
        return "Linear"

class FreeNote(FormationNote):
    @staticmethod
    def getName():
        return "Free formation"

class CubeNote(FormationNote):
    @staticmethod
    def getName():
        return "Cube formation"
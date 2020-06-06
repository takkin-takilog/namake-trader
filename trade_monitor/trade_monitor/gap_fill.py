import pandas as pd


class GapFill():

    def __init__(self, inst_id: int) -> None:

        self.__inst_id = inst_id

    @property
    def inst_id(self):
        return self.__inst_id

    @inst_id.setter
    def inst_id(self, inst_id):
        self.__inst_id = inst_id

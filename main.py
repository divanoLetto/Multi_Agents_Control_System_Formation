import matplotlib
matplotlib.use('TkAgg')
from tkApplication import *
import tkinter as tk  # python 2.7
import setting

def main():
    setting.init()
    makeTkApplication()

if __name__ == "__main__":
    main()
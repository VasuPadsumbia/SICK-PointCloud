# ------------------------------------------------
# --- Author: Farogh Iftekhar
# ------------------------------------------------
''' Description: This file return paths and other helper actions'''

from distutils.log import error
import os


#------------------------------------------------------------
# File/Folder path Configuration
#------------------------------------------------------------

def log_path():
    ''' It returns the log file path'''
    try:
        CWD = os.getcwd() #get current directory
        temp = os.path.join(CWD, "Logs")
        logs_path = os.path.join(temp, "application.log")
        return logs_path
        
    except OSError as error :
        print(error)


def smallMap_path():
    '''It return the smallMap_path file path'''
    try:
        CWD = os.getcwd() #get current directory
        temp = os.path.join(CWD, "maps")
        smallMap_path = os.path.join(temp, "map6_small.json")
        return smallMap_path
        
    except OSError as error :
        print(error)

def polarMap_path():
    '''It return the polarMap_path file path'''
    try:
        CWD = os.getcwd() #get current directory
        temp = os.path.join(CWD, "maps")
        polarMap_path = os.path.join(temp, "input5_small_polar_20deg.json")
        return polarMap_path
        
    except OSError as error :
        print(error)
import logging
import sys
import os
import json
from Layers import helper

class Singleton_meta(type):
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton_meta, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class Logger(metaclass = Singleton_meta):
    def __init__(self):
        '''Logger configurations, Debug could be enable disable using config file'''
        
        self.configPath = helper.configPath()
        self.loggerPath = os.path.join(helper.logsDir(), "logs.log")
        
        self.formatter = logging.Formatter(
             '[%(asctime)s] %(levelname)-8s %(name)-12s %(message)s')

        # Reading Debug config setting
        try:
            with open(self.configPath, "r") as json_file:
                data = json.load(json_file)
                self.debug = data['Debug']
                json_file.close()
        except:
            print("Error reading Config-JSON!")


    def getLogger(self, logname):
        ''' get logger returns the logger object, everything above
        debug would be saved in the file (Info, Warning, Critical)
        along with a timestamp. The method returns a logger instance which 
        shall be used for logging. Except debug, everythings gets logs in file.
        
        logging.debug('This is a debug message')
        logging.info('This is an info message')
        logging.warning('This is a warning message')
        logging.error('This is an error message')
        logging.critical('This is a critical message')'''
        

        logger = logging.getLogger(logname)
        logger.setLevel(logging.DEBUG)

        # Declare handlers
        fileHandler = logging.FileHandler(self.loggerPath)
        fileHandler.setLevel(logging.INFO)
        fileHandler.setFormatter(self.formatter)

        stremHandler = logging.StreamHandler(sys.stdout)
        stremHandler.setFormatter(self.formatter)

        # Adding handles
        logger.addHandler(fileHandler)
        logger.addHandler(stremHandler)

        # setting Debug Option
        if self.debug:
            logging.disable(logging.NOTSET)
        else:
            # if false disable Debug
            logging.disable(logging.DEBUG)

        return logger
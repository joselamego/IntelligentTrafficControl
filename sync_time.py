import logging
import ntplib
from time import strftime, localtime
from os import system
logging.basicConfig(filename='log', format='%(asctime)s %(message)s', datefmt='%m/%d/%Y %I:%M:%S %p', level=logging.DEBUG)

def run():
    try:
        client = ntplib.NTPClient()
        response = client.request('north-america.pool.ntp.org')
        system('date ' + strftime('%m%d%H%M%Y.%S',localtime(response.tx_time)))
        logging.info('Time synchronized with north-america.pool.ntp.org server.')

    except:
        logging.warning('Could not synchronize local time with ntp server.')

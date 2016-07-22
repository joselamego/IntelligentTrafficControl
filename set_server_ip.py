import logging
from subprocess import check_output
logging.basicConfig(filename='log', format='%(asctime)s %(message)s',
                    datefmt='%m/%d/%Y %I:%M:%S %p', level=logging.DEBUG)


def run():
    try:
        server_ip = check_output(
            "ifconfig wlan0 | awk \'/t addr:/{gsub(/.*:/,\"\",$2);print$2}\'",
            shell=True)
        server_ip = server_ip.replace("\n", "")
        logging.info('Server IP defined as: ' + str(server_ip))
        return server_ip
    except:
        logging.warning('Server IP cannot be defined.')

#!/usr/bin/env python

import os
import sys
import time
import subprocess

interfaceVpn = 'tun'
interface3g = 'CDC'
interfaceWifi = 'IEEE 802.11'
interfaceEther = 'eqos'


def getVpnIp():
    with open(os.devnull, 'w') as devnull:
        p = subprocess.check_output(('ip', 'tuntap', 'show'), stderr=devnull)
    
    p = p.decode("utf-8")

    if 'tun0' in p:   
        ip = None
        p = subprocess.check_output(('ip', 'addr', 'show', 'dev', 'tun0'))
        #print(p)
        p = p.decode("utf-8")
    
        for line in p.split("inet "):        
            if interfaceVpn in line:
                #print (line.strip())
                ip = line.split()[0].split()[0].split('/')[0]
                #print (ip)
    
        return ip

def get3gIp():
    ip = None
    p = subprocess.check_output(('lshw 2> /dev/null'), shell=True)
    #print(p)
    p = p.decode("utf-8")
    
    for line in p.split("\n"):        
        if interface3g in line:
            try:
                ip = line.split('ip=',1)[1].split()[0]
            except:
                pass
                    
            #print (ip)

    return ip


def getEtherIp():
    ip = None
    p = subprocess.check_output(('lshw 2> /dev/null'), shell=True)
    p = p.decode("utf-8")

    for line in p.split("\n"):
        if interfaceEther in line:
            try:
                ip = line.split('ip=',1)[1].split()[0]
            except:
                pass

    return ip


def getWifiIp():
    ip = None
    p = subprocess.check_output(('lshw 2> /dev/null'), shell=True)
    p = p.decode("utf-8")
    
    for line in p.split("\n"):        
        if interfaceWifi in line:
            #print (line.strip())
            ip = line.split('ip=',1)[1].split()[0]
            #print (line.split('ip=',1)[1].split()[0])
            
    return ip


def getActiveIp():
    ip = getVpnIp()
    
    if not ip:
        ip = get3gIp()

    if not ip:
        ip = getWifiIp()
    
    if not ip:
        ip = getEtherIp()
    
    if not ip:
        #ip = "localhost"    
        ip = "127.0.0.1"

    return ip


if __name__ == "__main__":
    ip = getActiveIp()
    print(ip)
    
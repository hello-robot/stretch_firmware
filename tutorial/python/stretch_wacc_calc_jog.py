#!/usr/bin/env python

import sys
from wacc_calc import WaccCalc
import argparse

parser=argparse.ArgumentParser(description='Comnmand and query the Wacc (Wrist Accelerometer) board from the keyboard')
args=parser.parse_args()

w=WaccCalc()
w.startup()



def menu():
    print('------ MENU -------')
    print('m: menu')
    print('r: reset board')
    print('a: set D2 on')
    print('b: set D2 off')
    print('c: set D3 on')
    print('d: set D3 off')
    print('X: do calculation')
    print '-------------------'

def step_interaction():
    menu()
    x=sys.stdin.readline()
    if len(x)>1:
        if x[0]=='m':
            menu()
        if x[0] == 'a':
            w.set_D2(1)
        if x[0] == 'b':
            w.set_D2(0)
        if x[0] == 'c':
            w.set_D3(1)
        if x[0] == 'd':
            w.set_D3(0)
        if x[0]=='r':
            print 'Resetting Board. Exiting...'
            w.board_reset()
            w.push_command()
            exit()

        if x[0]=='X':
            print '---Calculate Op(Var1,Var2) ---'
            print 'Op=0: Add'
            print 'Op=1: Mult'
            print 'Op=2: Div'
            print 'Enter Op'
            op=float(sys.stdin.readline())
            print 'Enter Var1'
            var1 = float(sys.stdin.readline())
            print 'Enter Var2'
            var2 = float(sys.stdin.readline())
            w.calculate(op,var1,var2)

        w.push_command()
    else:
        w.pull_status()
        w.pretty_print()

try:
    while True:
        try:
            step_interaction()
        except (ValueError):
            print('Bad input...')
except (KeyboardInterrupt, SystemExit):
    w.stop()

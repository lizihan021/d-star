#!/usr/bin/env python

import d_star as ds
#import d_star_lite as dsl

def d_star():
    s_last = s_start
    ds.initialize()
    path = computerShortestPath()
    while s_start != s_goal:
        #if g(s_start) = infinity, thre is no known path
        #s_start = argmin_{s in Succ(s_start)}(c(s_start,s') + g(s'))
        


def d_star_lite():
    print 'Oops'

def main():
    perform_d_star = True
    if d_star:
        d_star()
    else:
        #d_star_lite()

if __name__ == '__main__':
    main()

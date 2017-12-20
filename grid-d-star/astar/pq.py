#!/usr/bin/env python
# -*- coding: utf-8 -*-
from Queue import PriorityQueue

#defines a basic node class
class Node:
    def __init__(self,x_in,y_in,theta_in, id_in, parentid_in, cost_in):
        self.x = x_in
        self.y = y_in
        self.theta = theta_in
        self.id = id_in
        self.parentid = parentid_in
        self.cost = cost_in

    def printme(self):
        print "\tNode id", self.id,":", "x =", self.x, "y =",self.y, "theta =", self.theta, "parentid:", self.parentid

def pqComparator(lhs,rhs):
	return lhs[0] > rhs[0]
    # if lhs[0][0] > rhs[0][0]:
    #     return True
    # elif lhs[0][0] == rhs[0][0]:
    #     return lhs[0][1] > rhs[0][1]
    # else:
    #     return False

#### pq requires items pushed in to be unique. ###
class Priority_Queue:
	def __init__(self, compare = pqComparator, arr = []):
		# [key, value]
		self.pq = [[-1, -1]] + arr
		self.dic = dict(zip([x[1] for x in arr],[x[0] for x in arr]))
		self.compare = compare
		if not self.Empty():
			for i in range(len(self.pq)//2)[:0:-1]:
				fix_down(i)

	def put(self, key_item):
		self.Insert(key_item[1], key_item[0])
	def get(self):
		return self.Pop()

	def Top(self):
		if self.Empty():
			raise ValueError("Cannot get top(), Queue is empty.")
		return self.pq[1]

	def Pop(self):
		if self.Empty():
			raise ValueError("Cannot get top(), Queue is empty.")
		self.pq[1], self.pq[-1] = self.pq[-1], self.pq[1]
		victim = self.pq.pop()
		#self.dic.pop(victim[1], "dict value error")
		self.fix_down(1)
		return victim

	def Insert(self, item, key):
		self.pq.append([key, item])
		#self.dic[item] = key
		self.fix_up(self.size())

	def Update(self, item, new_key):
		idx = self.pq.index([self.dic[item], item])
		self.pq[idx][0] = new_key
		self.dict[item] = new_key
		self.fix_down(self.fix_up(idx))

	def Remove(self, item):
		idx = self.pq.index([self.dic[item], item])
		self.pq[idx], self.pq[-1] = self.pq[-1], self.pq[idx]
		victem = self.pq.pop()
		self.dic.pop(victem[1], "dict value error")
		self.fix_down(idx)
		return victem

	def fix_up(self, i):
		current = i
		while current > 1 and self.compare(self.pq[current//2], self.pq[current]):
			self.pq[current//2], self.pq[current] = self.pq[current], self.pq[current//2]
			current = current // 2
		return current

	def fix_down(self, i):
	    if 2*i <= self.size() and self.compare(self.pq[i], self.pq[2*i]):
	    	largest = 2*i
	    else:
	    	largest = i
	    if (2*i+1) <= self.size() and self.compare(self.pq[largest], self.pq[2*i + 1]):
	    	largest = 2*i + 1
	    if largest != i:
	    	self.pq[largest], self.pq[i] = self.pq[i], self.pq[largest]
	    	self.fix_down(largest)

	def Empty(self):
		return len(self.pq) == 1

	def Exist(self, item):
		if item in self.dic:
			return True
		else:
			return False

	def size(self):
		return len(self.pq) - 1

	def __str__(self):
		return self.pq.__str__()
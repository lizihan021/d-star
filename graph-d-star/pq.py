#!/usr/bin/env python

#### pq requires items pushed in to be unique. ###
def pqComparator(lhs,rhs):
    if lhs[0][0] > rhs[0][0]:
        return True
    elif lhs[0][0] == rhs[0][0]:
        return lhs[0][1] > rhs[0][1]
    else:
        return False


class Priority_Queue:
	def __init__(self, compare, arr = []):
		# [key, value]
		self.pq = [[-1, -1]] + arr
		self.dic = dict(zip([x[1] for x in arr],[x[0] for x in arr]))
		self.compare = compare
		if not self.empty():
			for i in range(len(self.pq)//2)[:0:-1]:
				fix_down(i)

	def Top(self):
		if self.empty():
			raise ValueError("Cannot get top(), Queue is empty.")
		return self.pq[1]

	def Pop(self):
		if self.empty():
			raise ValueError("Cannot get top(), Queue is empty.")
		self.pq[1], self.pq[-1] = self.pq[-1], self.pq[1]
		victim = self.pq.pop()
		self.dic.pop(victim[1], "dict value error")
		self.fix_down(1)
		return victim

	def Insert(self, item, key):
		self.pq.append([key, item])
		self.dic[item] = key
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

	def empty(self):
		return len(self.pq) == 1

	def exist(self, item):
		if item in self.dic:
			return True
		else:
			return False

	def size(self):
		return len(self.pq) - 1

	def __str__(self):
		return self.pq.__str__()

# if __name__ == "__main__":
# 	pq = Priority_Queue(pqComparator)
# 	pq.Insert("item", [1,2])
# 	pq.Insert("item2", [2,2])
# 	pq.Insert("item3", [7,2])
# 	pq.Insert("item4", [6,2])
# 	pq.Insert("item4", [6,2])
# 	pq.Insert("item4", [6,2])
# 	pq.Insert("item4", [6,2])
# 	pq.Insert("item5", [4,2])
# 	pq.Insert("item6", [9,2])
# 	print pq
# 	print pq.Pop()
# 	print pq
# 	print pq.Pop()
# 	pq.Remove("item")
# 	pq.Remove("item5")
# 	print pq

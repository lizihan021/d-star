
class priority_queue:
	def __init__(self, compare = lambda x, y: (x[0] < y[0]), arr = []):
		# [key, value]
		self.pq = [[-1, -1]] + arr
		self.dic = dict(zip([x[1] for x in arr],[x[0] for x in arr]))
		self.compare = compare
		if not self.empty():
			for i in range(len(self.pq)//2)[:0:-1]:
				fix_down(i)

	def top(self):
		if self.empty():
			raise ValueError("Cannot get top(), Queue is empty.")
		return self.pq[0][0]

	def pop(self):
		if self.empty():
			raise ValueError("Cannot get top(), Queue is empty.")
		self.pq[1], self.pq[-1] = self.pq[-1], self.pq[1]
		victem = self.pq.pop()
		self.dic.pop(victem[1], "dict value error")
		self.fix_down(1)
		return victem

	def push(self, item, key):
		self.pq.append([key, item])
		self.dic[item] = key
		self.fix_up(self.size())

	def update(self, item, new_key):
		idx = self.pq.index([self.dict[item], item])
		self.pq[idx][0] = new_key
		self.dict[item] = new_key
		self.fix_down(self.fix_up(idx))

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

	def Print(self):
		print self.pq

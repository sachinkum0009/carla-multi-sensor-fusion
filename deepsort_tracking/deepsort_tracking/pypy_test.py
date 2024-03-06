import time
import pycuda

t1 = time.time()
nums = range(100000000)
sum = 0
for k in nums:
    sum = sum + k
print("Sum of 1,000 numbers is : ", sum)
t2 = time.time()
t = t2 - t1
print("Elapsed time is : ", t, " seconds")
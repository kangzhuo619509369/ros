#递归函数

def digui(num):
    #1.打印num变量
    print(num)
    #2.检测num是否大于0
    if num > 0:
        #将num -1 传递给函数
        digui(num - 1)
    else:
        #输出一行横线
        print('----------')
    #3.再次打印num变量
    print(num)

#1.局部变量：每次调用的变量都对当前调用有效
#2.函数中num是否有过变化？ num每次都是第一次和第三次的输出一直

#调用函数
digui(3)

'''
def digui(3):
    #1.打印num变量
    print(3)
    #2.检测num是否大于0
    if 3 > 0:->True
        #将num -1 传递给函数
        digui(3 - 1)# 调用digui（2）函数
   
    #3.再次打印num变量
    print(3) 应该输出3,必须等待digui（2）完成


def digui(2):
    #1.打印num变量
    print(2)
    #2.检测num是否大于0
    if 2 > 0:
        #将num -1 传递给函数
        digui(2 - 1)# 调用digui(1)
  
    #3.再次打印num变量
    print(2) 应该输出2，必须等待digui(1)完成
    
调用digui（1）
def digui(1):
    #1.打印num变量
    print(1)
    #2.检测num是否大于0
    if 1 > 0:
        #将num -1 传递给函数
        digui(1 - 1)# 调用digui(0)
   
    #3.再次打印num变量
    print(1)  应该输出1，要等到digui(0)完成
    
#调用digui(0)
def digui(0):
    #1.打印num变量
    print(0)
    #2.检测num是否大于0
    if 0 > 0: ->False
       
    else:
        #输出一行横线
        print('----------')
    #3.再次打印num变量
    print(0)
'''
'''
3
2
1
0
-----------
0
1
2
3
'''


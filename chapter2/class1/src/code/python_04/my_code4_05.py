#收集参数
'''
#定义一个函数（计算3个数值的和的函数）
def getsum(no1,no2,no3):
    result = no1 + no2 + no3
    print(result)

#调用函数
getsum(5,7,9)
'''
'''
#定义一个函数（计算任意个数值的和的函数）
def getsum(*allnum):
    #声明一个累加和的变量
    total = 0
    #计算所有数值的和
    for i in allnum:
        total = total + i
    #打印求和结果
    print(total)


#调用函数
getsum(2,4,7,5,5,8,3,10)
'''

#注意事项
def getarg(a,b,c,*d):
    print(a,b,c)
    print(d)

#调用函数
getarg(1,2,3,4,5,6,7,8,9,10)
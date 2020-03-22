#查看函数的文档
#1.使用help函数(推荐使用)
#help(id)
#help(print)
#help(type)

#2.函数名.__doc__
#print(id.__doc__)
#print(print.__doc__)

#自定函数
def getsum(no1,no2):
    '''
    :功能:  计算两个数值的和
    :参数1  no1: Number类型
    :参数2  no2: Number类型
    :返回值 : 两个数值的和 Number
    '''
    result = no1 + no2
    return result

#查看自定义函数的文档
help(getsum)
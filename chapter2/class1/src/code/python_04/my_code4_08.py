#定义一个计算两个数值的和的函数
def getsum(no1,no2):
    result = no1 + no2
    #print(result)
    #返回值语句
    return result

#调用函数
num = getsum(12,23)
print(num)
print(num)
print(num)
#将求和的结果进行乘以10的操作
print(num * 10)

#查看return语句的另外一个作用
def test():
    print('************')
    #添加return语句
    return
    print('############')

#调用函数
test()


#测试print函数
result = print('@')
print(result)
'''
#函数（人形态）
#定义一个求和的函数
def getsum(no1,no2):
    result = no1 + no2
    return result

#调用函数
var = getsum(5,9)
print(var)
'''

'''
#lambda表达式（汽车）
getsum2 = lambda no1,no2 : no1 + no2 #lambda表达式自带return

#调用lambda表达式（函数）
var = getsum2(33,55)
print(var)
'''

#带有分支结构的lambda表达式
'''
def getsex(sex):
    #检测性别返回单词
    if sex == '男':
        return 'man'
    else:
        return 'woman'

#调用函数~
result = getsex('女')
print(result)
'''

getsex2 = lambda sex : 'man' if sex == '男' else   'woman'
#调用函数
result = getsex2('女')
print(result)


#在lambda表达式中调用它其他函数
#获取并且打印数据类型的函数  print(type(数据))

printtype = lambda var : print(type(var))

#调用函数
printtype('小明')
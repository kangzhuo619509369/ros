#定义函数

'''
#基本函数
#定义了一个输出一句话的函数
def mysun():
    print('太阳带给我们爱与温暖')

#特征：不调用不会执行
#调用函数
mysun()
mysun()
mysun()
mysun()

#查看函数的数据类型(函数的定义相当于变量赋值，函数名就是变量名)
print(type(mysun))
print(mysun)

'''

#带有参数的函数
def printnum(maxnum):
    var = 0
    while var <= maxnum:
        print(var)
        var += 1

#输出0-10之间的数字
#printnum()

#输出0-10的数字
#printnum(10)

#输出0-5的数字
#printnum(5)

#形参：在定义函数的括号中声明的变量名就是形参，形式上的参数
#实参：在调用函数的括号中传入的就是实参，实际上的参数
#注意：实参将值传递给形参的过程，本质上就是变量赋值操作
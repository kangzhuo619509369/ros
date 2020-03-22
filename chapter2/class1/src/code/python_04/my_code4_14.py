'''
#页面

#定义一个函数
def outer():
    #定义一个变量
    var = 55
    #定义一个内部函数
    def inner():
        #声明外部变量
        nonlocal var#当前变量var 不是全局变量也不是局部变量
        #在此处使用outer内部定义的var变量
        #在此处进行 +1 操作
        var = var + 1
        print(var)

    #调用inner函数
    inner()


#调用outer函数
outer()
'''


#python2的解决方案
#页面

#定义一个函数 【使用列表的穿透效果】
def outer():
    #定义一个变量（使用列表）
    var = [55]
    #定义一个内部函数
    def inner():

        #在此处使用outer内部定义的var变量
        #在此处进行 +1 操作
        var[0] = var[0] + 10
        print(var[0])

    #调用inner函数
    inner()


#调用outer函数
outer()


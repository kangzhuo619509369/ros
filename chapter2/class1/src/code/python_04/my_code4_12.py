#定义全局变量(仅仅可以进行访问操作，不可以进行其他操作)
mzj = 99#自以为这是完整的全局变量

#定义函数
def zui():
    #全局化声明
    global mzj
    #尝试修改全局变量
    mzj = mzj + 1
    print('在嘴巴中拔牙')
    print(mzj)


#调用函数
zui()

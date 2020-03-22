#多种参数混合存在的情况
#形参：普通形参 var  ,收集参数 *var, 关键字收集参数 **var
#实参: 普通实参  99  ，关键字参数 var = 99

#定义一个测试使用的函数
def testargs(name,sex,age,*size,**goodfriends):
    #使用普通参数
    print(name,sex,age)
    #使用收集参数
    print(size)
    #使用关键字收集参数
    print(goodfriends)

#调用函数
testargs('小明','男',18,178,125,35,gf1 = '小刚',gf2 = '小欧',gf3 = '小红')

#实参：普通实参在前，关键字参数在后

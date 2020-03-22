#自动类型转换
#运算情况
#声明一个整数
intvar = 99
#声明一个浮点数
floatvar = 5.56

#加法运算(发生了自动类型转换！ 自动的操作)
result = intvar + floatvar #intvar 转换成了浮点数 99 -> 99.0
print(result)
print(type(result))

#判断情况
if -5:#整型转化为布尔值
    print('python大法好')
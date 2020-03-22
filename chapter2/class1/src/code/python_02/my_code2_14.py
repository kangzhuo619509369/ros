#身份检测运算 检测数据的标识
#id标识  系统为数据分配的编号
var = 99
bs = id(var)
print(bs)

#is 检测是同一个
var1 = 'daxigua'
var2 = 'daxigua'
result = var1 is var2
print(result)
#查看id标识
print(id(var1),id(var2))



#注意：3.7以上版本 整数浮点数和字符串只要值一样，id标识就一样

#is not 检测是否不是同一个
var1 = 99
var2 = 98
result =  var1 is not var2
print(result)
#查看id标识
print(id(var1),id(var2))


#其他数据类型  列表，元组，字典，集合 数据相同，id标识也不同
var1 = [1,2,3]
var2 = [1,2,3]
result = var1 is var2
print(result)
#查看id标识
print(id(var1),id(var2))

#连续赋值的列表，元组，字典集合id标识相同
var1 = var2 = [2,3]
result = var1 is var2
print(result)
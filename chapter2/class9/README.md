# 2.9 元祖和字典

## 1. 元组的操作

### 元组的遍历

**for ... in**

```
变量 = (值1,值2,值3...)
for 变量 in 元组:
    使用变量获取元组的每个值
```

```python
# t = (1,2,3,4)
# for val in t:
#     print(val)
```

**while**

```
i = 0
while i<len(元组):
    使用元组变量[i]访问每个值
    i += 1
```

```python
# t = (1,2,3,4)
# i = 0
# while i < len(t):
#     print(t[i])
#     i += 1
```

**遍历长度相同的多级元组**

```
元组 = ((值1,值2...),(值1,值2...)...)
for 变量1，变量2.. in 元组:
    使用变量1和变量2
```

```python
# t = ((1,2),(3,4),(5,6))
# for val,var in t:
#     print(val,var)
```

**遍历长度不同的多级元组**

```
元组 = ((值1,值2...),(值1,值2...)...)
for 变量1 in 元组:
    for 变量2 in 变量1：
        使用变量2获取每个值
```

```python
# t = ((1,2),(3,4,5),(6,7))
# for val in t:
#     for var in val:
#         print(val,var)
```

### 元组的操作方法

**index()**

```python
获取指定值在元组中的索引值
格式：元组.index(值)
返回值：整数
print(t.index(2))
```

**count()**

```python
计算某个值在元组中出现的次数
格式：元组.count(值)
返回值：整数
print(t.count(2))
```

### 元组推导式

**基本格式：**

```
格式： 变量 = (i for i in 元组)
结果：不是元组而是一个生成器
```

**带条件格式：**

```
格式： 变量 = (i for i in 元组 if 条件表达式)
结果：不是元组而是一个生成器
```

**多循环推导式：**

```
格式： 变量 = (x+y for x in 元组1 for y in 元组2)
结果：不是元组而是一个生成器   x+y可以是其他操作
```

**带条件的多循环推导式：**

```
格式： 变量 = (x+y for x in 元组1 for y in 元组2 if 条件表达式)
结果：不是元组而是一个生成器   x+y可以是其他操作
```

## 2. 字典

**创建多个元素的字典**

```python
方式1：
    变量 = {键:值,键:值....}
   	dict1 = {"a":1,"b":2,"c":3}

方式2：
    变量 = dict({键:值,键:值....})
    dict2 = dict({"a":1,"b":2,"c":3})

方式3：
    变量 = dict(键=值,键=值...)
    注意：该方式键作为形参名使用，不可以添加引号，必须符合变量规则
    dict3 = dict(a=1,b=2,c=3)

方式4：
    变量 = dict([(键,值),(键,值)...])
    变量 = dict([[键,值],[键,值]...])
    变量 = dict(((键,值),(键,值)...))
    list1 = ["a","b","c"]
    list2 = [1,2,3]
    dict4 = dict(zip(list1,list2))

    list1 = ["a","b","c","d","e","f"]
    list2 = [1,2,3]
    dict4 = dict(zip(list1,range(1,10)))

方式5：
    变量 = dict(zip((键，键...),(值,值...)))
```

### 字典的遍历

**遍历键**

```
for 变量i in 字典:
    #使用i遍历所有的键，有键就可以通过变量访问其值
```

```python
# for key in dict1:
#     print(key)
```

**遍历键和值**

```
for 变量i，变量j in 字典.items():
    #使用变量i遍历所有键，通过变量j遍历所有值
```

```python
# for key,value in dict1.items():
#     print(key,value)
```

### 字典的内置函数

**clear()**

```
功能：清空字典
格式：字典.clear()
返回值：None
注意：直接改变原有字典
```

```python
# dict1.clear()
# print(dict1)
```

**copy()**

```
功能：复制字典
格式：字典.copy()
返回值：新的字典
```

```python
# dict2 = dict1.copy()
# print(dict2)
```

**fromkeys()**

```
功能：使用指定的序列作为键创建字典
格式：字典.fromkeys(序列,值)
返回值：字典
```

```python
# lsd = ["d","e","f"]
# lsd1 = [1,2,3]
# dict2 = dict1.fromkeys(lsd,lsd1)
# print(dict2)
```

**get()**

```
功能：根据键获取指定的值
格式：字典.get(键[，默认值])
返回值:值
注意:如果键不存在，则使用默认值,如果没有默认值则返回None
```

```python
# print(dict1["e"])
# print(dict1.get("e"))
```

**items()**

```
功能：将字典的键值转化成类似元组的形式方便遍历
格式：字典.items()
返回值：类似元组的类型
```

**keys()**

```
功能：将字典的所有键组成一个序列
格式：字典.keys()
返回值:序列
```

```python
print(dict1.keys())
```

**values()**

```
功能：将字典的所有值组成一个序列
格式：字典.values()
返回值：序列
```

```python
print(dict1.values())
```

**pop()**

```
功能：移除字典中指定的元素
格式：字典.pop(键[,默认值])
返回值：被移除的键对应的值
注意：如果键不存在，则报错，如果键不存在，默认值设置，则返回默认值
```

```python
# dict2 = dict1.pop("a")
# print(dict2)
```

**popitem()**

```
功能：移除字典中的键值对
格式：字典.popitem()
返回值：键值对组成的元组
注意：弹一个原字典就少一个，字典为空就不可以弹出，会报错
```

```python
# dict1.popitem()
# dict1.popitem()
# dict1.popitem()
# dict1.popitem()
# print(dict1)
```

**setdefault()**

```
功能：添加一个元素
格式：字典.setdefault(键,值)
返回值:None
注意：添加是键存在则不进行任何操作，键不存在则添加，添加是不写值，默认None值
```

```python
# dict1.setdefault("d",4)
# print(dict1)
#
# dict1["e"] = 5
# print(dict1)
```

**update()**

```
功能：修改字典中的值

方式1：
    格式： 字典.update(键=值)
    返回值：None
    注意:此处的键作为关键字参数使用，不可以添加''

方式2：
    格式: 字典.update({键:值})
    返回值：None
```

```python
# dict1.update(d=5)
# print(dict1)
#
# dict1.update({"e":6})
# print(dict1)

# 如果添加的key在原始字典中没有  那么就添加新的键值对   如果有  那么就更新原字典的键所对应的值
# dict2 = {"f":7,"g":8,"a":0}
# dict1.update(dict2)
# print(dict1)
```




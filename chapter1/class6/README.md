# 1.6 Shell

Shell是用户与内核进行交互操作的一种接口，是Linux最重要的软件之一。目前最流行的Shell称为bash，bash脚本编程以其简洁、高效而著称，多年来成为Linux程序员和系统管理员解决实际问题的利器。Shell是操作系统的最外层。Shell合并编程语言以控制进程和文件，以及启动和控制其它程序。Shell通过提示您输入，向操作系统解释该输入，然后处理来自操作系统的任何结果输出来管理您与操作系统之间的交互。Shell提供了与操作系统通信的方式。此通信以交互的方式（来自键盘的输入立即操作）或作为一个 Shell脚本执行。Shell 脚本是 Shell 和操作系统命令的序列，它存储在文件中。当登录到系统中时，系统定位要执行的Shell的名称。在它执行之后，Shell显示一个命令提示符。普通用户的此提示符通常是一个`$`（美元符）。当提示符下输入命令并按下Enter键时，Shell对命令进行求值，并尝试执行它。取决于命令说明，Shell 将命令输出写到屏幕或重定向到输出。然后它返回命令提示符，并等待您输入另一个命令。命令行是输入所在的行。它包含Shell提示符。每行的基本格式如下：

```bash
$ 命令 参数（一个或多个）
```

Shell视命令行的第一个字（直到第一个空白空格）为命令，所有后继字为自变量。bash脚本第一句是`#！/bin/bash`，`#！`是一个约定的标记，它告诉系统这个脚本需要`/bin/bash`程序作为解释器。

## 1. Shell字符

字符串可以用单引号`‘’`，也可以用双引号`“”`，也可以不用引号。

单引号字符串
- 单引号里的任何字符都会原样输出，单引号字符串中的变量是无效的；
- 单引号字符串中不能出现单引号（对单引号使用转义符也不行）

双引号字符串
- 双引号里的内容可以有变量
- 双引号里可以出现转义字符

拼接字符串： 表示使用多个变量与字符串拼接的变量.

获取字符串长度： `${#’变量名’}`表示对应变量名的字符长度

提取字符串：`${‘变量名’:2:5}`表示从’变量’字符串中的第3个字符开始截取5个字符串

查找字符串：expr index “变量” “查找的字符串” 会根据字符串的位置进行输出

## 2. Shell数组
- bash支持一维数组，不支持多维，并且没有限定数组的大小。
- 数组元素的下标由`0`开始编号。获取数组中的元素要利用下标，下标可以是整数或算数表达式，应大于或等于`0`。

定义数组

用括号来表示数组。数组元素用“空格”符号分割开。定义数字的一般形式为

直接定义 : 数组名=(值1 值2 …值x) | 各个分量定义: 数组名[0]=xxx 数组名[1]=xxx 数组名[2]=xxx

```bash
a=(1 2 3 4)
```

读取数组

读取数组元素值格式`${数组名[下标]}`
```bash
echo ${a[2]}

3
```
`${数组名[@]}与${数组名[*]}`表示获取数组名对应的所有元素

```bash
echo ${a[@]}
1 2 3 4

echo ${a[*]}
1 2 3 4
```

获取数组长度

获取数组长度的方法与获取字符串长度的方法相同

```bash
length=${#数组名[@]}

或

length=${#数组名[*]}
```

```bash
length=${#a[@]}
echo $length

4
```

获取数组中单个元素的长度

```bash
length=${#数组名[n]}
```

```bash
length=${#a[1]}
echo $length 

1
```

## 3. 变量

**变量类型**
1、局部变量：局部变量在脚本或命令中定义，仅在当前shell实例中有效。其他shell启动的程序不能访问局部变量。

2、环境变量：所有的程序，包括shell启动的程序，都能访问环境变量。有些程序需要环境变量来保证其正常运行。必要的时候shell脚本也可以自定义环境变量。

3、shell变量：shell变量时由shell程序设置的特殊变量。shell变量中有一部分是环境变量，有一部分是局部变量。这些变量保证了shelll的正常运行。

**定义变量**

export 赋值

```bash
a=”test”
```

变量名和等号之间不能有空格。以及如下规则
● 首个字符必须为英文大小写字母“ a-z、A-Z ”
● 中间不能有空格，可以使用下划线 “ _ “
● 不能使用标点符号
● 不能使用bash里的关键字。（即变量名不可以与其他命令相同）

**使用变量**

使用定义过的变量,只要在变量名前加`$`即可。

```bash
name=’test’
echo $name
echo ${name}
```

```bash
b="hello"
echo $b
hello

echo ${b}
hello
```

变量名的大括号为可选项.为了方便解释器识别变量的边界

{}标识括号内的值为变量

**只读变量**

使用readonly 命令可以将变量属性定义只读.

```bash
name=’test’
readonly name
```

如果再次赋值 name 变量,就会报错。

```bash
name="test"
realonly name

name="hello"
bash: name: 只读变量
```

**删除变量**

unset命令可以删除变量，直接`unset “变量名”`

例：

`unset “a”`

删除变量a，a不在是变量

```bash
name=luoyunxiang
echo $name

unset name
echo $name
```

## 4. for循环

```bash
#！ /bin/bash
for x(变量）in `expr 1 100`（变量的范围）
do (循环语句的开始）
        …… （需要循环的命令）
done （循环语句结束）

#！ /bin/bash
for ((表达式；表达式；表达式))
do (循环语句的开始）
        …… （需要循环的命令）
done （循环语句结束）

for ((i=1(赋给i的原始值为1）;i<10（变量i的值小于10）;i++（每次循环一次，变量就加一，步进值为1）))
for ((i=1(赋给i的原始值为1）;i<10（变量i的值小于10）;i=i+2（每次循环一次，变量就加二，步进值为2）))
for ((i=1(赋给i的原始值为1）;（表达式为空则无限循环）;i++（每次循环一次，变量就加一，步进值为1）))
```

```bash
#!/bin/bash
for number in 1 2 3 4 5
do
        echo $number
done
exit 0
```

```bash
bash for_demo.bash
1
2
3
4
5
```

## 5. if循环

```bash
if （条件1）
	then （动作1）
	elif （条件2）
		then （动作2）

        	…… （命令）
		else （匹配不上其他条件后匹配else）
        	（else的动作）
fi （条件语句的结束）
```

```bash
#!/bin/bash
age=21
if [ $age -gt 18 ]
then
  echo "你成年了"
fi
```

```bash
bash if_demo.bash

你成年了
```

## 6. while循环

也称为前测试循环语句，重复次数是利用一个条件来控制是否继续重复执行这个语句。为了避免死循环，必须保证循环体中包含循环出口条件即表达式存在退出状态为非0的情况。

```bash
while [ condition ] 
do
	（需要循环的命令）
done
```

打印Welcome信息5次。

```bash
#!/bin/bash
x=1
while [ $x -le 5 ]
do
  echo "Welcome $x times"
  x=$(( $x + 1 ))
done
```
```bash
bash while_demo1.bash
Welcome 1 times
Welcome 2 times
Welcome 3 times
Welcome 4 times
Welcome 5 times
```

无限循环

```bash
#！ /bin/bash

while :
do(循环语句的开始）
        …… （需要循环的命令）
done(循环语句的结束）
```

```bash
#！ /bin/bash

while true
do(循环语句的开始）
        …… （需要循环的命令）
done(循环语句的结束）
```

无限循环打印Welcome。

```bash
#!/bin/bash
x=1
while true
do
  echo "Welcome $x times"
  x=$(( $x + 1 ))
done
```
```bash
Welcome 73851 times
Welcome 73852 times
Welcome 73853 times
Welcome ^C
```

下面是一个递归执行while循环的例子。

```bash
#!/bin/bash
counter=$1
factorial=1
while [ $counter -gt 0 ]
do
   factorial=$(( $factorial * $counter ))
   counter=$(( $counter - 1 ))
done
echo $factorial
```

## 7. until 循环

until执行一系列的操作直至条件为真时停止，条件可以为任意测试条件，并且测试发生在循环末尾。因此至少执行一次。

```bash
until （循环条件）
do (循环语句的开始）
	…… （需要循环的命令）
done (循环语句的结束）
```

```bash
#!/bin/bash
 
i=0
 
until [[ "$i" -gt 5 ]]    #大于5
do
    let "square=i*i"
    echo "$i * $i = $square"
    let "i++"
done
```

```bash
bash until_demo.bash

0 * 0 = 0
1 * 1 = 1
2 * 2 = 4
3 * 3 = 9
4 * 4 = 16
5 * 5 = 25
```

## 8. bash脚本命令

| 命令                         | 解释                                                         | 例         |
| ---------------------------- | ------------------------------------------------------------ | ---------- |
| source a.sh                  | 本进程中执行shell脚本a.sh                                        |            |
| ./a.sh                       | 执行shell脚本a.sh<br/>前提：有执行权限足够<br/>执行权限添加：chmod U+x a.sh ||
| env                          | 查看全部环境变量                                             |            |
| export                       | 添加环境变量                                                 |            |
| expr                         | 添加本地变量                                                 |            |
| read                         | 用户手动赋值                                                 |            |
| read -p“提示是信息”          | 用户手动进行赋值，引号里面加的是提示信息                     |            |
| pstree                       | 查看进程树                                                   |            |
| sh -n                        | 后面跟shlle脚本文件名，检查编辑的shlle脚本是否有语法错误     |            |
| let                          | 做算术运算                                                   | let a=$i+1<br>运算$i+1的值赋给a |
| break                        | 跳出当前循环                                                 |            |
| echo $?                      | 测试上条命令是否成功，回显0为成功，回显其他都为失败          |            |

## 9. 比较

| 命令             | 解释             | 例                                                           |
| ---------------- | ---------------- | ------------------------------------------------------------ |
| -ge              | 大于或等于       |                                                              |
| -gt              | 大于             |                                                              |
| -le              | 小于或等于       |                                                              |
| -lt              | 小于             |                                                              |
| -eq              | 等于             |                                                              |
| -f               | 判断文件是否存在 | #！ /bin/bash<br/>if [ -f $a.out ]<br/>then<br/>        echo $a.out<br/>fi |
| -d               | 判断目录是否存在 | #！ /bin/bash<br/>if [ -d $aaa ]<br/>then<br/>        echo $a.out |

## 10 练习1 

判断文件是不是块或字符设备文件，如果是将其拷贝到/dev目录下

相关知识：

1. read命令从键盘或文件中读入信息，将其赋给变量（一个或多个），直到遇到回车符或文件结束符为止。

2. I/O设备大致分为两类：块设备和字符设备。块设备将信息存储在固定大小的块中，每个块都有自己的地址。数据块的大小通常在512字节到32768字节之间。块设备的基本特征是每个块都能独立于其它块而读写。磁盘是最常见的块设备。

3. if 命令的语法是：

```bash
if TEST-COMMANDS; then CONSEQUENT-COMMANDS; fi
```

TEST-COMMAND执行后且它的返回状态是0，那么 CONSEQUENT-COMMANDS 就执行。返回状态是最后一个命令的退出状态，或者当没有条件是真的话为0。TEST-COMMAND 经常包括数字和字符串的比较测试，但是也可以是任何在成功时返回状态0或者失败时返回一些其他状态的一些命令。一元表达式经常用于检查文件的状态。如果对某个要素primaries， FILE 参数是 /dev/fd/N 这样的形式，那么就检查文件描述符 “N”。stdin, stdout 和 stderr 和他们各自的文件描述符也可以用于测试。

4. test 命令最短的定义可能是评估一个表达式；如果条件为真，则返回一个 0 值。如果表达式不为真，则返回一个大于 0 的值——也可以将其称为假值。检查最后所执行命令的状态的最简便方法是使用 $? 值。

5. cp命令把指定的源文件复制到目标文件或把多个源文件复制到目标目录中。

审核要求：

1. 代码规范性

2. 功能完备性、稳定可靠性

3. 问题考虑是否全面

4、程序交互性体验

```bash
read -p "input a file:" filename
if [ -b $filename -o -c $filename ]
then
    cp $filename /dev
fi
```



## 11 练习2 

模拟Linux登陆Shell

相关知识：

1. 使用echo命令可以显示文本行或变量，或者把字符串输入到文件。它的一般形式为`echo string`；`echo`命令一般不需用引号来标记字符串。如果字符串中有空格，引号等特殊字符，可以用引号将其括起来。否则输出结果会出问题。

2. if 命令的语法是：

```bash
if TEST-COMMANDS; then CONSEQUENT-COMMANDS; fi
```

TEST-COMMAND执行后且它的返回状态是0，那么CONSEQUENT-COMMANDS就执行。返回状态是最后一个命令的退出状态，或者当没有条件是真的话为0。TEST-COMMAND经常包括数字和字符串的比较测试，但是也可以是任何在成功时返回状态0或者失败时返回一些其他状态的一些命令。一元表达式经常用于检查文件的状态。如果对某个要素primaries， FILE 参数是 /dev/fd/N 这样的形式，那么就检查文件描述符 “N”。stdin, stdout 和 stderr 和他们各自的文件描述符也可以用于测试。

3. read命令从键盘或文件中读入信息，将其赋给变量（一个或多个），直到遇到回车符或文件结束符为止。

审核要求：

1. 代码规范性

2. 功能完备性、稳定可靠性

3. 问题考虑是否全面

4. 程序交互性体验

```bash
#/bin/bash

echo -n "login:"  

read name

echo -n "password:"

read passwd

if [ $name = "cht" -a $passwd = "abc" ];then

        echo "the host and password is right!"

else echo "input is error!"

fi
```

## 12. 练习3 

从键盘读取两个数，并比较两个数大小，并打印结果

相关知识：

1. 使用echo命令可以显示文本行或变量，或者把字符串输入到文件。它的一般形式为：`echo string`；echo命令一般不需用引号来标记字符串。如果字符串中有空格，引号等特殊字符，可以用引号将其括起来。否则输出结果会出问题。

2. if 命令的语法是：

```bash
if TEST-COMMANDS; then CONSEQUENT-COMMANDS; fi
```
TEST-COMMAND 执行后且它的返回状态是0，那么 CONSEQUENT-COMMANDS 就执行。返回状态是最后一个命令的退出状态，或者当没有条件是真的话为0。TEST-COMMAND 经常包括数字和字符串的比较测试，但是也可以是任何在成功时返回状态0或者失败时返回一些其他状态的一些命令。一元表达式经常用于检查文件的状态。如果对某个要素primaries， FILE 参数是 /dev/fd/N 这样的形式，那么就检查文件描述符 “N”。stdin, stdout 和 stderr 和他们各自的文件描述符也可以用于测试。

3. read命令从键盘或文件中读入信息，将其赋给变量（一个或多个），直到遇到回车符或文件结束符为止。

审核要求：

1. 代码规范性
2. 功能完备性、稳定可靠性
3. 问题考虑是否全面
4. 程序交互性体验

```bash
#/bin/bash
echo "please enter two number"
read a
read b
if test $a -eq $b
then echo "NO.1 = NO.2"
elif test $a -gt $b
then echo "NO.1 > NO.2"
else echo "NO.1 < NO.2" 
fi
```

## 13. 练习4 

判断用户是否在运行

相关知识：

1. if 命令的语法是

```bash
if TEST-COMMANDS; then CONSEQUENT-COMMANDS; fi
```

TEST-COMMAND 执行后且它的返回状态是0，那么 CONSEQUENT-COMMANDS 就执行。返回状态是最后一个命令的退出状态，或者当没有条件是真的话为0。TEST-COMMAND 经常包括数字和字符串的比较测试，但是也可以是任何在成功时返回状态0或者失败时返回一些其他状态的一些命令。一元表达式经常用于检查文件的状态。

2. whoami命令可以显示当前用户名称。

审核要求：

1. 代码规范性

2. 功能完备性、稳定可靠性

3. 问题考虑是否全面

4. 程序交互性体验

```bash
#/bin/bash

echo "Please enter a user:"
read a
b=$(whoami)
if test $a = $b
then echo "the user is running."
else echo "the user is not running."
fi
```
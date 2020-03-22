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

WK是一种处理文本文件的语言，是一个强大的文本分析工具。

之所以叫AWK是因为其取了三位创始人 Alfred Aho，Peter Weinberger, 和 Brian Kernighan 的 Family Name 的首字符。

## 10. awk

```
awk [选项参数] 'script' var=value file(s)
或
awk [选项参数] -f scriptfile var=value file(s)
```

**选项参数说明：**

-  -F fs or --field-separator fs
   指定输入文件折分隔符，fs是一个字符串或者是一个正则表达式，如-F:。 
-  -v var=value or --asign var=value
   赋值一个用户定义变量。 
-  -f scripfile or --file scriptfile
   从脚本文件中读取awk命令。 
-  -mf nnn and -mr nnn
   对nnn值设置内在限制，-mf选项限制分配给nnn的最大块数目；-mr选项限制记录的最大数目。这两个功能是Bell实验室版awk的扩展功能，在标准awk中不适用。 
-  -W compact or --compat, -W traditional or --traditional
   在兼容模式下运行awk。所以gawk的行为和标准的awk完全一样，所有的awk扩展都被忽略。 
-  -W copyleft or --copyleft, -W copyright or --copyright
   打印简短的版权信息。 
-  -W help or --help, -W usage or --usage
   打印全部awk选项和每个选项的简短说明。 
-  -W lint or --lint
   打印不能向传统unix平台移植的结构的警告。 
-  -W lint-old or --lint-old
   打印关于不能向传统unix平台移植的结构的警告。 
-  -W posix
   打开兼容模式。但有以下限制，不识别：/x、函数关键字、func、换码序列以及当fs是一个空格时，将新行作为一个域分隔符；操作符**和**=不能代替^和^=；fflush无效。 
-  -W re-interval or --re-inerval
   允许间隔正则表达式的使用，参考(grep中的Posix字符类)，如括号表达式[[:alpha:]]。 
-  -W source program-text or --source program-text
   使用program-text作为源代码，可与-f命令混用。 
-  -W version or --version
   打印bug报告信息的版本。

------

### 10.1 基本用法

log.txt文本内容如下：

```
2 this is a test
3 Are you like awk
This's a test
10 There are orange,apple,mongo
```

用法一：

```
awk '{[pattern] action}' {filenames}   # 行匹配语句 awk '' 只能用单引号
```

实例：

```
# 每行按空格或TAB分割，输出文本中的1、4项
 $ awk '{print $1,$4}' log.txt
 ---------------------------------------------
 2 a
 3 like
 This's
 10 orange,apple,mongo
 # 格式化输出
 $ awk '{printf "%-8s %-10s\n",$1,$4}' log.txt
 ---------------------------------------------
 2        a
 3        like
 This's
 10       orange,apple,mongo
 
```

用法二：

```
awk -F  #-F相当于内置变量FS, 指定分割字符
```

实例：

```
# 使用","分割
 $  awk -F, '{print $1,$2}'   log.txt
 ---------------------------------------------
 2 this is a test
 3 Are you like awk
 This's a test
 10 There are orange apple
 # 或者使用内建变量
 $ awk 'BEGIN{FS=","} {print $1,$2}'     log.txt
 ---------------------------------------------
 2 this is a test
 3 Are you like awk
 This's a test
 10 There are orange apple
 # 使用多个分隔符.先使用空格分割，然后对分割结果再使用","分割
 $ awk -F '[ ,]'  '{print $1,$2,$5}'   log.txt
 ---------------------------------------------
 2 this test
 3 Are awk
 This's a
 10 There apple
```

用法三：

```
awk -v  # 设置变量
```

实例：

```
 $ awk -va=1 '{print $1,$1+a}' log.txt
 ---------------------------------------------
 2 3
 3 4
 This's 1
 10 11
 $ awk -va=1 -vb=s '{print $1,$1+a,$1b}' log.txt
 ---------------------------------------------
 2 3 2s
 3 4 3s
 This's 1 This'ss
 10 11 10s
```

用法四：

```
awk -f {awk脚本} {文件名}
```

实例：

```
 $ awk -f cal.awk log.txt
```

------

### 10.2 运算符

| 运算符                  | 描述                             |
| ----------------------- | -------------------------------- |
| = += -= *= /= %= ^= **= | 赋值                             |
| ?:                      | C条件表达式                      |
| \|\|                    | 逻辑或                           |
| &&                      | 逻辑与                           |
| ~ 和 !~                 | 匹配正则表达式和不匹配正则表达式 |
| < <= > >= != ==         | 关系运算符                       |
| 空格                    | 连接                             |
| + -                     | 加，减                           |
| * / %                   | 乘，除与求余                     |
| + - !                   | 一元加，减和逻辑非               |
| ^ ***                   | 求幂                             |
| ++ --                   | 增加或减少，作为前缀或后缀       |
| $                       | 字段引用                         |
| in                      | 数组成员                         |

 过滤第一列大于2的行 

```
$ awk '$1>2' log.txt    #命令
#输出
3 Are you like awk
This's a test
10 There are orange,apple,mongo
```

过滤第一列等于2的行

```
$ awk '$1==2 {print $1,$3}' log.txt    #命令
#输出
2 is
```

过滤第一列大于2并且第二列等于'Are'的行

```
$ awk '$1>2 && $2=="Are" {print $1,$2,$3}' log.txt    #命令
#输出
3 Are you
```

------

### 10.3 内建变量

| 变量        | 描述                                                       |
| ----------- | ---------------------------------------------------------- |
| $n          | 当前记录的第n个字段，字段间由FS分隔                        |
| $0          | 完整的输入记录                                             |
| ARGC        | 命令行参数的数目                                           |
| ARGIND      | 命令行中当前文件的位置(从0开始算)                          |
| ARGV        | 包含命令行参数的数组                                       |
| CONVFMT     | 数字转换格式(默认值为%.6g)ENVIRON环境变量关联数组          |
| ERRNO       | 最后一个系统错误的描述                                     |
| FIELDWIDTHS | 字段宽度列表(用空格键分隔)                                 |
| FILENAME    | 当前文件名                                                 |
| FNR         | 各文件分别计数的行号                                       |
| FS          | 字段分隔符(默认是任何空格)                                 |
| IGNORECASE  | 如果为真，则进行忽略大小写的匹配                           |
| NF          | 一条记录的字段的数目                                       |
| NR          | 已经读出的记录数，就是行号，从1开始                        |
| OFMT        | 数字的输出格式(默认值是%.6g)                               |
| OFS         | 输出记录分隔符（输出换行符），输出时用指定的符号代替换行符 |
| ORS         | 输出记录分隔符(默认值是一个换行符)                         |
| RLENGTH     | 由match函数所匹配的字符串的长度                            |
| RS          | 记录分隔符(默认是一个换行符)                               |
| RSTART      | 由match函数所匹配的字符串的第一个位置                      |
| SUBSEP      | 数组下标分隔符(默认值是/034)                               |

```
$ awk 'BEGIN{printf "%4s %4s %4s %4s %4s %4s %4s %4s %4s\n","FILENAME","ARGC","FNR","FS","NF","NR","OFS","ORS","RS";printf "---------------------------------------------\n"} {printf "%4s %4s %4s %4s %4s %4s %4s %4s %4s\n",FILENAME,ARGC,FNR,FS,NF,NR,OFS,ORS,RS}'  log.txt
FILENAME ARGC  FNR   FS   NF   NR  OFS  ORS   RS
---------------------------------------------
log.txt    2    1         5    1
log.txt    2    2         5    2
log.txt    2    3         3    3
log.txt    2    4         4    4
$ awk -F\' 'BEGIN{printf "%4s %4s %4s %4s %4s %4s %4s %4s %4s\n","FILENAME","ARGC","FNR","FS","NF","NR","OFS","ORS","RS";printf "---------------------------------------------\n"} {printf "%4s %4s %4s %4s %4s %4s %4s %4s %4s\n",FILENAME,ARGC,FNR,FS,NF,NR,OFS,ORS,RS}'  log.txt
FILENAME ARGC  FNR   FS   NF   NR  OFS  ORS   RS
---------------------------------------------
log.txt    2    1    '    1    1
log.txt    2    2    '    1    2
log.txt    2    3    '    2    3
log.txt    2    4    '    1    4
# 输出顺序号 NR, 匹配文本行号
$ awk '{print NR,FNR,$1,$2,$3}' log.txt
---------------------------------------------
1 1 2 this is
2 2 3 Are you
3 3 This's a test
4 4 10 There are
# 指定输出分割符
$  awk '{print $1,$2,$5}' OFS=" $ "  log.txt
---------------------------------------------
2 $ this $ test
3 $ Are $ awk
This's $ a $
10 $ There $
```

------

### 10.4 使用正则，字符串匹配

```
# 输出第二列包含 "th"，并打印第二列与第四列
$ awk '$2 ~ /th/ {print $2,$4}' log.txt
---------------------------------------------
this a
```

**~ 表示模式开始。// 中是模式。**

```
# 输出包含"re" 的行
$ awk '/re/ ' log.txt
---------------------------------------------
3 Are you like awk
10 There are orange,apple,mongo
```

------

###  10.5 忽略大小写 

```
$ awk 'BEGIN{IGNORECASE=1} /this/' log.txt
---------------------------------------------
2 this is a test
This's a test
```

------

###  10.6 模式取反

```
$ awk '$2 !~ /th/ {print $2,$4}' log.txt
---------------------------------------------
Are like
a
There orange,apple,mongo
$ awk '!/th/ {print $2,$4}' log.txt
---------------------------------------------
Are like
a
There orange,apple,mongo
```

------

###  10.7 awk脚本

关于awk脚本，我们需要注意两个关键词BEGIN和END。

- BEGIN{ 这里面放的是执行前的语句 }
- END {这里面放的是处理完所有的行后要执行的语句 }
- {这里面放的是处理每一行时要执行的语句}

假设有这么一个文件（学生成绩表）： 

```
$ cat score.txt
Marry   2143 78 84 77
Jack    2321 66 78 45
Tom     2122 48 77 71
Mike    2537 87 97 95
Bob     2415 40 57 62
```

我们的awk脚本如下：

```
$ cat cal.awk
#!/bin/awk -f
#运行前
BEGIN {
    math = 0
    english = 0
    computer = 0
 
    printf "NAME    NO.   MATH  ENGLISH  COMPUTER   TOTAL\n"
    printf "---------------------------------------------\n"
}
#运行中
{
    math+=$3
    english+=$4
    computer+=$5
    printf "%-6s %-6s %4d %8d %8d %8d\n", $1, $2, $3,$4,$5, $3+$4+$5
}
#运行后
END {
    printf "---------------------------------------------\n"
    printf "  TOTAL:%10d %8d %8d \n", math, english, computer
    printf "AVERAGE:%10.2f %8.2f %8.2f\n", math/NR, english/NR, computer/NR
}
```

我们来看一下执行结果：

```
$ awk -f cal.awk score.txt
NAME    NO.   MATH  ENGLISH  COMPUTER   TOTAL
---------------------------------------------
Marry  2143     78       84       77      239
Jack   2321     66       78       45      189
Tom    2122     48       77       71      196
Mike   2537     87       97       95      279
Bob    2415     40       57       62      159
---------------------------------------------
  TOTAL:       319      393      350
AVERAGE:     63.80    78.60    70.00
```

------

**实例**

AWK的hello world程序为：

```
BEGIN { print "Hello, world!" }
```

 计算文件大小

```
$ ls -l *.txt | awk '{sum+=$6} END {print sum}'
--------------------------------------------------
666581
```

 从文件中找出长度大于80的行 

```
awk 'length>80' log.txt
```

 打印九九乘法表 

```
seq 9 | sed 'H;g' | awk -v RS='' '{for(i=1;i<=NF;i++)printf("%dx%d=%d%s", i, NR, i*NR, i==NR?"\n":"\t")}
```

## 11. sed查找与替换

### 11.1 在文本文件离进行替换

在很多 shell 脚本的工作都从通过 grep 或 egrep 去除所需的文本开始。正则表达式查找的最初结果，往往就成了要拿来作进一步处理的“原始数据”。通常，文本替换至少需要做一件事，就是讲一些字以另一些字取代，或者删除匹配行的某个部分。

执行文本替换的正确程序应该是 sed----流编辑器。
 sed 的设计就是用来批处理而不是交互的方式编辑文件。当药做好几个变化的时候，不管是对一个还是对数个文件，比较简单的方式就是将这些变更部分写到一个编辑的脚本里，再将此脚本应用到所有必须修改的文件，sed 的存在目的就在这里。

在 shell 里，sed 主要用于一些简单的文本替换，所以我们先从他开始。

基本用法：我们经常在管道中间使用 sed，用来执行替换操作，做法是使用 s 命令----要求正则表达式寻找，用替换文本替换匹配的文本呢，以及可选的标志：
 `sed ‘s’:.*//’  /etcpasswd  |`   删除第一个冒号之后所有的东西
 `sort -u`          排序列表并删除重复部分
 sed的语法：
 `sed [-n] ‘editing command’ [file...]`
 `sed [-n] -e ‘editing command’ [file...]`
 `sed [-n] -f script-file...  [file...]`
 用途: sed  可删除（delete）、改变（change）、添加（append）、插入（insert）、合、交换文件中的资料行，或读入其它档的资料到文>件中，也可替换（substuite）它们其中的字串、或转换（tranfer）其中的字母等等。例如将文件中的连续空白行删成一行、"local"字串替换成"remote"、"t"字母转换成"T"、将第  10 行资料与第 11 资料合等。
 总合上述所言，当 sed 由标准输入读入一行资料并放入 pattern space 时，sed 依照 sed script 的编辑指令逐一对  pattern space 内的资料执行编辑之後，再由 pattern space  内的结果送到标准输出，接着再将下一行资料读入。如此重执行上述动作，直至读完所有资料行为止。
 小结，记住：
 (1)sed 总是以行对输入进行处理
 (2)sed 处理的不是原文件而是原文件的拷贝

主要参数：  

- -e：执行命令行中的指令，例如`：sed -e 'command' file(s)`  
- -f：执行一个 sed 脚本文件中的指令，例如： `sed -f  scriptfile file(s)`  
- -i：与-e的区别在于：当使用-e 时，sed 执行指令并不会修改原输入文件的内容，只会显示在 bash 中，而使用-i 选项时，sed 执行的指令会直接修改原输入文件。  
- -n：读取下一行到 pattern space。

### 11.2 行为模式

读取输入文件的每一行。假如没有文件的话，则是标准输入。以每一行来说，sed 会执行每一个应用倒数入行的 esiting  command。结果会写到标准输出（默认情况下，或是显式的使用 p 命令及-n 选项）。若无-e 或-f 选项，则 sed  会把第一个参数看做是要使用的 editing command。

```
find  /home/tolstoy  -type -d -print // 寻找所有目录  
sed ‘s;/home/tolstor;/home/lt/;’ // 修改名称；注意：这里使用分号作为定界符  
sed ‘s/^/mkdir /’ `//插入 mkdir 命令  
sh -x`                           //以 shell 跟踪模式执行
```

上述脚本是说将/home/tolstoy 目录结构建立一份副本在/home.lt 下（可能是为备份）而做的准备

### 11.3 替换案例

Sed 可替换文件中的字串、资料行、甚至资料区。其中，表示替换字串的指令中的函数参数为s；表示替换资料行、或资料区>的指令中的函数参数为c。上述情况以下面三个例子说明。

*行的替换 sed -e '1c/#!/bin/more' file （把第一行替换成#!/bin/more） 思考：把第 n 行替换成 just do it sed -e 'nc/just do it' file sed -e '1,10c/I can do it' file  （把 1 到 10 行替换成一行： I can do it） 思考：换成两行（I can do it! Let's start） sed -e '1,10c/I can do it!/nLet'"/'"'s start' file* 字符的替换
 `$ sed 's/test/mytest/g' example`-----在整行范围内把 test 替换为 mytest。如果没有 g 标记，则只有每行第一个匹配的 test 被替换成 mytest。
 `$ sed -n 's/^test/mytest/p' example`-----（-n）选项和 p 标志一起使用表示只打印那些发生替换的行。也就是说，如果某一行开头的 test 被替换成 mytest，就打印它。
 `$ sed 's/^192.168.0.1/&localhost/' example`-----&符号表示替换换字符串中被找到的部份。所有以 192.168.0.1 开头的行都会被替换成它自已加 localhost，变成 192.168.0.1localhost。
 `$ sed -n 's/loveable/\1rs/p' example`-----love 被标记为 1，所有 loveable 会被替换成 lovers，而且替换的行会被打印出来。
 `$ sed 's#10#100#g' example`-----不论什么字符，紧跟着 s 命令的都被认为是新的分隔符，所以，“#”在这里是分隔符，代替了默认的“/”分隔符。表示把所有 10 替换成 100。

### 11.4 替换与查找

在 s 命令里以 g 结尾表示的是：全局性，意即“替代文本取代正则表达式中每一个匹配的”。如果没有设置 gsed 指挥取代第一个匹配的。

鲜为人知的是：可以在结尾指定数字，只是第 n 个匹配出现才要被取代：
 `sed ‘s/Tom/Lisy/2’ < Test.txt`   仅匹配第二个 Tom
 通过给 sed 增加一个-e 选项的方式能让 sed 接受多个命令。
 `sed -e ‘s/foo/bar/g’ -e ‘s/chicken/cow/g’  myfile.txt 1>myfile2.txt`
 用 shell 命令将 test.log 文件中第 3-5 行的第 2 个“filter”替换成“haha”
 `sed -i '3,5s/filter/haha/2' test.log`

下面介绍所有 sed 的函数参数的功能（editing command）。
 = 印出资料行数( line number )。  

- a ：添加使用者输入的资料。  
- b ：label 将执行的指令跳至由 : 建立的参考位置。  
- c ：以使用者输入的资料取代资料。  
- d ：删除资料。  
- D ：删除 pattern space 内第一个 newline 字母 前的资料。  
- g ：拷贝资料从 hold space。  
- G ：添加资料从 hold space 至 pattern space 。  
- h ：拷贝资料从 pattern space 至 hold space 。  
- H ：添加资料从 pattern space 至 hold space 。  
- l ：印出 l 资料中的 nonprinting character 用 ASCII 码。  
- i ：插入添加使用者输入的资料行。  
- n ：读入下一笔资料。  
- N ：添加下一笔资料到 pattern space。  
- p ：印出资料。  
- P ：印出 pattern space 内第一个 newline 字母 前的资料。  
- q ：跳出 sed 编辑。  
- r ：读入它档内容。  
- s ：替换字串。  
- w ：写资料到它档内。  
- x ：交换 hold space 与 pattern space 内容。  
- y ：转换（transform）字元。

## 12 练习1 

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



## 13 练习2 

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

## 14. 练习3 

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

## 15. 练习4 

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
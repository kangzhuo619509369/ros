# 1.5 程序调试

## 1. GDB

GDB是一个由GNU开源组织发布的、Unix/Linux操作系统下的、基于命令行的、功能强大的程序调试工具。GDB中的命令固然很多，但我们只需掌握其中十个左右的命令，就大致可以完成日常的基本的程序调试工作。

| 命令                                                         | 解释                                                         | 示例                                                         |
| ------------------------------------------------------------ | ------------------------------------------------------------ | ------------------------------------------------------------ |
| file                                                         | 加载被调试的可执行程序文件。因为一般都在被调试程序所在目录下执行GDB，因而文本名不需要带路径。 | (gdb) 			file gdb-sample                            |
| r                                                            | Run的简写，运行被调试的程序。如果此前没有下过断点，则执行完整个程序；如果有断点，则程序暂停在第一个可用断点处。 | (gdb) 			r                                          |
| c                                                            | Continue的简写，继续执行被调试程序，直至下一个断点或程序结束。 | (gdb) 			c                                          |
| b <行号> <br> b <函数名称> <br> b <函数名称> <br> b <代码地址> <br> d [编号] | b:Breakpoint的简写，设置断点。两可以使用“行号”“函数名称”“执行地址”等方式指定断点位置。其中在函数名称前面加“”，符号表示将断点设置在“由编译器生成的prolog代码处”。如果不了解汇编，可以不予理会此用法。<br>d: Delete breakpoint的简写，删除指定编号的某个断点，或删除所有断点。断点编号从1开始递增。 | (gdb) b 8 <br> (gdb) b main (gdb) <br> b \*main <br> (gdb) b \*0x804835c <br> (gdb) d |
| s,n                                                          | s: 执行一行源程序代码，如果此行代码中有函数调用，则进入该函数；<br>n:执行一行源程序代码，此行代码中的函数调用也一并执行。<br>s相当于其它调试器中的“Step Into (单步跟踪进入)”；n相当于其它调试器中的“StepOver (单步跟踪)”。这两个命令必须在有源代码调试信息的情况下才可以使用（GCC编译时使用“-g”参数）。 | <br> (gdb) s <br> (gdb) n                              |
| si,ni                                       | si命令类似于s命令，ni命令类似于n命令。所不同的是，这两个命令（si/ni）所针对的是汇编指令，而s/n针对的是源代码。 | <br> (gdb) si <br> (gdb) ni                           |
| p <变量名称>                         | Print的简写，显示指定变量（临时变量或全局变量）的值。 | (gdb) p i <br> (gdb) p nGlobalVar                 |
| display <br> undisplay <编号> | display，设置程序中断后欲显示的数据及其格式。例如，如果希望每次程序中断后可以看到即将被执行的下一条汇编指令，可以使用命令“display /i $pc 其中 $pc 代表当前汇编指令， /i 表示以十六进行显示。当需要关心汇编代码时，此命令相当有用。 undispaly，取消先前的display设置，编号从1开始递增。 | <br> (gdb) display /i $pc 			(gdb) 			undisplay 1 |
| i                                                        | Info的简写，用于显示各类信息，详情请查阅“help i”。 | <br> (gdb) i r                                     |
| q                                                        | Quit的简写，退出 GDB调试环境。                | <br> (gdb) q                                      |
| help [命令名称]                      | GDB帮助命令，提供对GDB名种命令的解释说明。如果指定了“命令名称”参数，则显示该命令的详细说明；如果没有指定参数，则分类显示所有GDB命令，供用户进一步浏览和查询。| <br> (gdb) help display                           |

例子，gdb-sample.c

```c
#include <stdio.h>

int nGlobalVar = 0;

int tempFunction(int a, int b)
{
    printf("tempFunction is called, a = %d, b = %d /n", a, b);
    return (a + b);
}

int main()
{
    int n;
    n = 1;
    n++;
    n--;

    nGlobalVar += 100;
    nGlobalVar -= 12;

    printf("n = %d, nGlobalVar = %d /n", n, nGlobalVar);

    n = tempFunction(1, 2);
    printf("n = %d", n);

    return 0;
}
```

```bash
gcc gdb-sample.c -o gdb-sample -g
```

在上面的命令行中，使用 -o 参数指定了编译生成的可执行文件名为gdb-sample，使用参数 -g 表示将源代码信息编译到可执行文件中。如果不使用参数 -g，会给后面的GDB调试造成不便。当然，如果我们没有程序的源代码，自然也无从使用 -g 参数，调试/跟踪时也只能是汇编代码级别的调试/跟踪。

```
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "x86_64-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word".
(gdb) 
```

上面最后一行`(gdb)`为GDB内部命令引导符，等待用户输入GDB命令。

```
(gdb) file gdb-sample 
Reading symbols from gdb-sample...done.
```

上面最后一行提示已经加载成功。

```
(gdb) r
Starting program: /home/lyx/demo/gdb-sample 
n = 1, nGlobalVar = 88 /ntempFunction is called, a = 1, b = 2 /nn = 3[Inferior 1 (process 19999) exited normally]
```

```
(gdb) b main
Breakpoint 1 at 0x40055d: file demo/demo2.c, line 14.
```

上面最后一行提示已经成功设置断点，并给出了该断点信息。在源文件`gdb-sample.c`第14行处设置断点；这是本程序的第一个断点，序号为1；断点处的代码地址为0x40055d（此值可能仅在本次调试过程中有效）。看源代码，第14行中的代码为`n= 1`，恰好是main函数中的第一个可执行语句，前面的“`int n;`为变量定义语句，并非可执行语句）

![figure_5_1](src/images/figure_5_1.png)

```
(gdb) r
Starting program: /home/lyx/demo/gdb-sample 

Breakpoint 1, main () at demo/demo2.c:14
14	    n = 1;
```

 程序中断在`gdb-sample.c`第14行处，即`main`函数是第一个可执行语句处。最后一行信息为下一条将要执行的源代码为`n = 1;`，它是源代码文件gdb-sample.c中的第14行。

```
(gdb) s
15	    n++;
```

上面的信息表示已经执行完`n= 1;`，并显示下一条要执行的代码为第20行的`n++;`。

```
(gdb) p n
$3 = 1
```

`n=1`，$1表示这是第一次使用`p`命令，再次执行`p n`将显示`$2= 1`，$3表示这是第三次使用`p`命令。

```
(gdb) b 20
Breakpoint 2 at 0x40058a: file demo/demo2.c, line 20.

(gdb) b tempFunction
Breakpoint 3 at 0x400534: file demo/demo2.c, line 7.
```

```
(gdb) c
Continuing.

Breakpoint 2, main () at demo/demo2.c:21
21	    printf("n = %d, nGlobalVar = %d /n", n, nGlobalVar);
(gdb) p nGlobalVar 
$4 = 88

(gdb) c
Continuing.

Breakpoint 3, tempFunction (a=1, b=2) at demo/demo2.c:7
7	    printf("tempFunction is called, a = %d, b = %d /n", a, b);
(gdb) p a
$5 = 1
(gdb) p b
$6 = 2

(gdb) c
Continuing.
n = 1, nGlobalVar = 88 /ntempFunction is called, a = 1, b = 2 /nn = 3[Inferior 1 (process 20313) exited normally]
```

`c`跳到断点或程序结束，程序正常退出。

有时候需要看到编译器生成的汇编代码，以进行汇编级的调试或跟踪，又该如何操作呢？

```
(gdb) display /i $pc
1: x/i $pc
<error: No registers.>
(gdb) r
Starting program: /home/lyx/demo/gdb-sample 

Breakpoint 1, main () at demo/demo2.c:14
14	    n = 1;
1: x/i $pc
=> 0x40055d <main+8>:	movl   $0x1,-0x4(%rbp)
```

看到了汇编代码，`n= 1;`”对应的汇编代码是`movl   $0x1,-0x4(%rbp)`。

```assembly
(gdb) si
15	    n++;
1: x/i $pc
=> 0x400564 <main+15>:	addl   $0x1,-0x4(%rbp)
(gdb) si
16	    n--;
1: x/i $pc
=> 0x400568 <main+19>:	subl   $0x1,-0x4(%rbp)
(gdb) si
18	    nGlobalVar += 100;
1: x/i $pc
=> 0x40056c <main+23>:	
    mov    0x200aca(%rip),%eax        # 0x60103c <nGlobalVar>
(gdb) si
0x0000000000400572	18	    nGlobalVar += 100;
1: x/i $pc
=> 0x400572 <main+29>:	add    $0x64,%eax
```

si命令类似于s命令，ni命令类似于n命令。所不同的是，这两个命令所针对的是汇编指令，而s/n针对的是源代码。

接下来我们试一下命令`b <函数名称>`。

```
(gdb) d
删除所有断点吗？ (y or n) y
(gdb) 
```

当被询问是否删除所有断点时，输入`y`并按回车键即可。

断点设置在main函数入口，`step in`显示对应下条程序对应的汇编代码。

```assembly
(gdb) b *main
Breakpoint 4 at 0x400555: file demo/demo2.c, line 12.
(gdb) r
The program being debugged has been started already.
Start it from the beginning? (y or n) y
Starting program: /home/lyx/demo/gdb-sample 

Breakpoint 4, main () at demo/demo2.c:12
12	{
1: x/i $pc
=> 0x400555 <main>:	push   %rbp
(gdb) si
0x0000000000400556	12	{
1: x/i $pc
=> 0x400556 <main+1>:	mov    %rsp,%rbp
(gdb) si
0x0000000000400559	12	{
1: x/i $pc
=> 0x400559 <main+4>:	sub    $0x10,%rsp
(gdb) si
14	    n = 1;
1: x/i $pc
=> 0x40055d <main+8>:	movl   $0x1,-0x4(%rbp)
(gdb) si
15	    n++;
1: x/i $pc
=> 0x400564 <main+15>:	addl   $0x1,-0x4(%rbp)
(gdb) si
16	    n--;
1: x/i $pc
=> 0x400568 <main+19>:	subl   $0x1,-0x4(%rbp)
(gdb) si
18	    nGlobalVar += 100;
1: x/i $pc
=> 0x40056c <main+23>:	
    mov    0x200aca(%rip),%eax        # 0x60103c <nGlobalVar>
```

```assembly
(gdb) i r
rax            0x400555	4195669
rbx            0x0	0
rcx            0x0	0
rdx            0x7fffffffd9e8	140737488345576
rsi            0x7fffffffd9d8	140737488345560
rdi            0x1	1
rbp            0x7fffffffd8f0	0x7fffffffd8f0
rsp            0x7fffffffd8e0	0x7fffffffd8e0
r8             0x400650	4195920
r9             0x7ffff7de7ac0	140737351940800
r10            0x846	2118
r11            0x7ffff7a2d740	140737348032320
r12            0x400430	4195376
r13            0x7fffffffd9d0	140737488345552
r14            0x0	0
r15            0x0	0
rip            0x40056c	0x40056c <main+23>
eflags         0x202	[ IF ]
cs             0x33	51
ss             0x2b	43
ds             0x0	0
es             0x0	0
fs             0x0	0

(gdb) i r eax
eax            0x400555	4195669

(gdb) q
A debugging session is active.

	Inferior 1 [process 20709] will be killed.

Quit anyway? (y or n) y
```

`(gdb) i r`显示寄存器中的当前值。`(gdb) i r eax`为显示寄存器`eax`存储的值。`(gdb) q`退出。

**练习1. 用gdb发现eg.c程序错误。**

这个程序将运行 10 次 for 循环，使用 'wib()" 函数计算出累积值，最后打印出结果。程序代码如下

```c
#include <stdio.h> 

int wib(int no1, int no2)
{
  int result, diff;
  diff = no1 - no2;
  result = no1 / diff;
  return result;
}

int main(int argc, char *argv[])
{
  int value, div, result, i, total;
  value = 10;
  div = 6;
  total = 0;
  for(i = 0; i < 10; i++)
  {
    result = wib(value, div);
    total += result;
    div++;
    value--;
  }
  printf("%d wibed by %d equals %d\n", value, div, total);
  return 0;
}
```

1. 编译

```
gcc -g eg.c -o eg
```

2. 运行gdb

```c
$ gdb
GNU gdb (Ubuntu 7.11.1-0ubuntu1~16.5) 7.11.1
Copyright (C) 2016 Free Software Foundation, Inc.
License GPLv3+: GNU GPL version 3 or later <http://gnu.org/licenses/gpl.html>
This is free software: you are free to change and redistribute it.
There is NO WARRANTY, to the extent permitted by law.  Type "show copying"
and "show warranty" for details.
This GDB was configured as "x86_64-linux-gnu".
Type "show configuration" for configuration details.
For bug reporting instructions, please see:
<http://www.gnu.org/software/gdb/bugs/>.
Find the GDB manual and other documentation resources online at:
<http://www.gnu.org/software/gdb/documentation/>.
For help, type "help".
Type "apropos word" to search for commands related to "word".
(gdb) fie eg
Undefined command: "fie".  Try "help".
(gdb) 
```


在`gdb`中，使用 `file`命令装入要调试的程序，例`file eg`。装入程序后，用命令`run` 来启动程序。使用`run`运行程序会产生以下消息：

```
(gdb) run
Starting program: /home/lyx/eg 

Program received signal SIGFPE, Arithmetic exception.
0x000000000040053d in wib (no1=8, no2=8) at demo4.c:7
7	  result = no1 / diff;
```

gdb指出在程序第7行发生一个算术异常`Arithmetic exception`，通常它会打印这一行以及 wib() 函数的自变量值。要查看第 7 行前后的源代码，请使用`list`命令，它通常会打印 10 行。再次输入 'list'（或者按回车重复上一条命令）将列出程序的下 10 行。从 gdb  消息中可以看出，第 7 行中的除法运算出了错，程序在这一行中将变量 `no1`除以`diff`。

```c
(gdb) list
6	
7		int count = 0;
8	
9		printf("\n");
10	
11		for(i=0; i<6; i++){
12	
13			for(j=0; j<11; j++){
14	
15				if((i>j) || (10-j)<i){

(gdb) list
16	
17					printf(" ");
18	
19				}else{
20	
21					fflush(stdout);
22	
23					printf("*");	
24	
25					count++;
```

要查看变量的值，使用`print`或`p`命令并指定变量名。输入`print no1`和`print diff`，可以相应看到 `no1`和 `diff`的值，结果如下

```c
(gdb) p no1
$1 = 8
(gdb) p diff
$2 = 0
(gdb) p no1-no2
$3 = 0
```

gdb指出`no1`等于8，`diff`等于0。根据这些值和第7行中的语句，我们可以推断出算术异常是由除数为 0 
的除法运算造成的。清单显示了第6行计算的变量`diff`，我们可以打印`diff`表达式（使用`p no1-no2`命令），来重新估计这个变量。gdb告诉我们wib函数的这两个自变量都等于 8，于是我们要检查调用wib()函数的main()函数，以查看这是在什么时候发生的。在允许程序自然终止的同时，我们使用`continue`命令告诉 gdb 继续执行。

```c
(gdb) c
Continuing.

Program terminated with signal SIGFPE, Arithmetic exception.
The program no longer exists.
```

GNU调试器是所有程序员工具库中的一个功能非常强大的工具。本讲只介绍了gdb的一小部分功能。要了解更多知识，建议您阅读GNU调试器手册。

### 1.1 GDB分析实例1

```cpp
#include <iostream>
using namespace std;  

int divint(int, int);  
int main() 
{ 
   int x = 5, y = 2; 
   cout << divint(x, y); 
   
   x =3; y = 0; 
   cout << divint(x, y); 
   
   return 0; 
}  

int divint(int a, int b) 
{ 
   return a / b; 
}   
```

[Debug讲解](./sample1.md)

### 1.2 GDB分析实例2

```cpp
#include<iostream>
 
using namespace std;
 
long factorial(int n);
 
int main()
{
    int n(0);
    cin>>n;
    long val=factorial(n);
    cout<<val;
    cin.get();
    return 0;
}
 
long factorial(int n)
{
    long result(1);
    while(n--)
    {
        result*=n;
    }
    return result;
}
```

[Debug讲解](./sample2.md)

## 2. 内存错误和内存泄漏

virgrind可以用来检测程序开发中的绝大多数内存，函数、缓存使用、多线程竞争访问内存、堆栈问题，是一个Linux下功能非常强大内存检测工具。

- valgrind-tool=<name> 最常用的选项。运行valgrind中名为toolname的工具。默认memcheck。

- memcheck:这是valgrind应用最广泛的工具，一个重量级的内存检查器，能够发现开发中绝大多数内存错误问题，比如：使用未初始化的内存，使用已经释放了的内存，内存访问越界等。下面将重点介绍此功能。

- callgrind: 主要用来检查程序中函数调用过程中出现的问题。

- cachegrind: 主要用来检查程序中缓存使用出现的问题。

- helgrind: 主要用来检查多线程程序中出现的竞争问题。

- massif: 主要用来检查程序中堆栈使用中出现的问题。

- extension: 可以利用core提供的功能，自己编写特定的内存调试工具。


### 2.1 valgrind memcheck

#### 2.1.1 memcheck内存检测原理

**valid-value表**
对于进程的整个地址空间中的每一个字节(byte)，都有与之对应的8个bits，对于CPU的每个寄存器，也有一个与之对应的bit向量。这些bits负责记录该字节或者寄存器值是否具有有效的、已经初始化的值。

**valid-Address表**
对于进程整个地址空间中的一个字节(byte)，还有与之对应的1bit，负责记录该地址是否能够被读写。

**内存检测原理**
当要读写内存中的某个字节时，首先检查这个字节对应的address  bit。如果该address  bit显示该位置是无效位置，memcheck则报告内存读写错误。valgrind内核相当于一个虚拟的CPU环境，当内存中的某个字节被加载到真实的CPU中时，该字节对应的value  bit也被加载到虚拟的CPU环境中，一旦寄存器中的值，被用来产生内存地址，或者该值能够影响程序的输出，则mencheck会检查对应的value  bits，如果该值尚未初始化，则会报告使用未初始化内存错误。

### 2.2 memcheck内存检测

#### 2.2.1源码

创建gdbmem.cpp源码文件，准备待检测的代码如下：

```cpp
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <malloc.h>
#include<pthread.h>
#include<string.h>

//memory access overflow
char* stringcopy(char* psrc)
{
    int len = strlen(psrc) + 1;
    char* pdst = (char*)malloc(len);//12
    memset(pdst, 0, len * 2);//13
    memcpy(pdst, psrc, len*2);//14
    return pdst;
}

//array assess overflow
void printarray(int arry[], int arrysize)
{
    int i = 0;
    for(; i < arrysize; i++)
    //for(i = arrysize-1; i >= 0; i--)
    {
        printf("arry[%d]:%d\n",i, arry[i]);
    }
    printf("arry[%d]:%d\n",i+1, arry[i+1]);//27
}

//main body
int main(int narg, const char** args)
{
    char* pwildptr;
    char* pstr = "this is a memory debug program!\n";
    int array[10] = {1,2,3,4,5,6,7,8,9,10};
    char* ptmp = stringcopy(pstr);//36
    //memory leak
    char* ptmp2 = (char*)malloc(100);//38
    memset(ptmp2, 0, 100);
    // memory write overflow
    printf(ptmp);//41
    // array tip assess overflow
    printarray(array, 10);//43
    free(ptmp);//44
    printf("%p", pwildptr);//45
    //wild ptr copy
    memcpy(ptmp, ptmp2, 20);//47
    printf(ptmp);//48
    return 0;
}
```

#### 2.2.2 编译

```shell
g++ -o gdbmem gdbmem.o
```

编译后将在当前目录下生成gdbmem可执行文件。

#### 2.2.3 valgrind内存检测

valgring 对gdbmem进行内存检测

```shell
$ valgrind --tool=memcheck --leak-check=full --track-fds=yes ./gdbmem
==10668== Memcheck, a memory error detector
==10668== Copyright (C) 2002-2015, and GNU GPL'd, by Julian Seward et al.
==10668== Using Valgrind-3.11.0 and LibVEX; rerun with -h for copyright info
==10668== Command: ./gdbmem
==10668== 
==10668== Invalid write of size 8
==10668==    at 0x4C3453F: memset (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x40075D: stringcopy(char*) (gdbmem.cpp:13)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204060 is 32 bytes inside a block of size 33 alloc'd
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid write of size 1
==10668==    at 0x4C34558: memset (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x40075D: stringcopy(char*) (gdbmem.cpp:13)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204080 is 16 bytes after a block of size 48 in arena "client"
==10668== 
==10668== Invalid write of size 8
==10668==    at 0x4C326CB: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400778: stringcopy(char*) (gdbmem.cpp:14)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204060 is 32 bytes inside a block of size 33 alloc'd
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid write of size 2
==10668==    at 0x4C32723: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400778: stringcopy(char*) (gdbmem.cpp:14)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204080 is 16 bytes after a block of size 48 in arena "client"
==10668== 
this is a memory debug program!
arry[0]:1
arry[1]:2
arry[2]:3
arry[3]:4
arry[4]:5
arry[5]:6
arry[6]:7
arry[7]:8
arry[8]:9
arry[9]:10
arry[11]:-623874025
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E8890E: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Use of uninitialised value of size 8
==10668==    at 0x4E84711: _itoa_word (_itoa.c:180)
==10668==    by 0x4E8812C: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E84718: _itoa_word (_itoa.c:180)
==10668==    by 0x4E8812C: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E881AF: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E87C59: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E87CE2: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
```

main函数45行，memcpy(ptmp, ptmp2, 20);读取已经被释放的内存，导致memcheck报错。

```
==10668== Invalid write of size 8
==10668==    at 0x4C326CB: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008FE: main (gdbmem.cpp:47)
==10668==  Address 0x5204040 is 0 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid write of size 2
==10668==    at 0x4C32723: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008FE: main (gdbmem.cpp:47)
==10668==  Address 0x5204050 is 16 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid read of size 1
==10668==    at 0x4ED0760: strchrnul (strchr.S:24)
==10668==    by 0x4E87207: __find_specmb (printf-parse.h:108)
==10668==    by 0x4E87207: vfprintf (vfprintf.c:1312)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x40090F: main (gdbmem.cpp:48)
==10668==  Address 0x5204040 is 0 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid read of size 1
==10668==    at 0x4E8741A: vfprintf (vfprintf.c:1324)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x40090F: main (gdbmem.cpp:48)
==10668==  Address 0x5204040 is 0 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
0x40097d==10668== 
==10668== FILE DESCRIPTORS: 3 open at exit.
==10668== Open file descriptor 2: /dev/pts/4
==10668==    <inherited from parent>
==10668== 
==10668== Open file descriptor 1: /dev/pts/4
==10668==    <inherited from parent>
==10668== 
==10668== Open file descriptor 0: /dev/pts/4
==10668==    <inherited from parent>
==10668== 
==10668== 
==10668== HEAP SUMMARY:
==10668==     in use at exit: 100 bytes in 1 blocks
==10668==   total heap usage: 3 allocs, 2 frees, 1,157 bytes allocated
==10668== 
==10668== 100 bytes in 1 blocks are definitely lost in loss record 1 of 1
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400888: main (gdbmem.cpp:38)
==10668== 
==10668== LEAK SUMMARY:
==10668==    definitely lost: 100 bytes in 1 blocks
==10668==    indirectly lost: 0 bytes in 0 blocks
==10668==      possibly lost: 0 bytes in 0 blocks
==10668==    still reachable: 0 bytes in 0 blocks
==10668==         suppressed: 0 bytes in 0 blocks
==10668== 
==10668== For counts of detected and suppressed errors, rerun with: -v
==10668== Use --track-origins=yes to see where uninitialised values come from
==10668== ERROR SUMMARY: 37 errors from 15 contexts (suppressed: 0 from 0)
```

#### 2.2.3 memcheck检测结果分析

- memcheck的LEAK SUMMARY输出结果将内存泄漏分为以下几种情况：
- definitely lost:明确地已经泄漏了，因为在程序运行完的时候，没有指针指向它, 指向它的指针在程序中丢失了
- indirectly lost:间接丢失。当使用了含有指针成员的类或结构时可能会报这个错误。这类错误无需直接修复，他们总是与”definitely lost”一起出现，只要修复”definitely lost”即可。
- possibly lost:发现了一个指向某块内存中部的指针，而不是指向内存块头部。这种指针一般是原先指向内存块头部，后来移动到了内存块的中部，还有可能该指针和该内存根本就没有关系，检测工具只是怀疑有内存泄漏。
- still reachable:可以访问，未丢失但也未释放
- suppressed:已被解决。出现了内存泄露但系统自动处理了。可以无视这类错误。
- 内存泄漏概述：

```shell
==10668== LEAK SUMMARY:
==10668==    definitely lost: 100 bytes in 1 blocks
==10668==    indirectly lost: 0 bytes in 0 blocks
==10668==      possibly lost: 0 bytes in 0 blocks
==10668==    still reachable: 0 bytes in 0 blocks
==10668==         suppressed: 0 bytes in 0 blocks
```

此处只有100个字节的内存泄漏。

```shell
==10668== Invalid write of size 8
==10668==    at 0x4C3453F: memset (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x40075D: stringcopy(char*) (gdbmem.cpp:13)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204060 is 32 bytes inside a block of size 33 alloc'd
```

根据错误提示，stringcopy 函数13行，即memset(pdst, 0, len  *2);申请了len的数据长度，memset的时候却使用了2*len的数据长度，内存写溢出了。

```
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid write of size 1
==10668==    at 0x4C34558: memset (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x40075D: stringcopy(char*) (gdbmem.cpp:13)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204080 is 16 bytes after a block of size 48 in arena "client"
```

 stringcopy 函数13行，即memset(pdst, 0, len*2);申请了len的数据长度，memset的时候却使用了2*len的数据长度，内存写溢出。相同语句的内存写溢出，却报了两个错误，原因笔者目前也还没有弄明白，如果有大虾指点，不胜感激。

```
==10668== 
==10668== Invalid write of size 8
==10668==    at 0x4C326CB: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400778: stringcopy(char*) (gdbmem.cpp:14)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204060 is 32 bytes inside a block of size 33 alloc'd
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid write of size 2
==10668==    at 0x4C32723: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400778: stringcopy(char*) (gdbmem.cpp:14)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668==  Address 0x5204080 is 16 bytes after a block of size 48 in arena "client"
```

stringcopy 函数13行，即memcpy(pdst, psrc, len*2);申请了len的数据长度，memset的时候却使用了2*len的数据长度，内存写溢出。

```
==10668== 
this is a memory debug program!
arry[0]:1
arry[1]:2
arry[2]:3
arry[3]:4
arry[4]:5
arry[5]:6
arry[6]:7
arry[7]:8
arry[8]:9
arry[9]:10
arry[11]:-623874025
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E8890E: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Use of uninitialised value of size 8
==10668==    at 0x4E84711: _itoa_word (_itoa.c:180)
==10668==    by 0x4E8812C: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E84718: _itoa_word (_itoa.c:180)
==10668==    by 0x4E8812C: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E881AF: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E87C59: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
==10668== 
==10668== Conditional jump or move depends on uninitialised value(s)
==10668==    at 0x4E87CE2: vfprintf (vfprintf.c:1631)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x4008E6: main (gdbmem.cpp:45)
```

main函数45行，printf("%p", pwildptr);读取未初始化的野指针

```
==10668== 
==10668== Invalid write of size 8
==10668==    at 0x4C326CB: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008FE: main (gdbmem.cpp:47)
==10668==  Address 0x5204040 is 0 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid write of size 2
==10668==    at 0x4C32723: memcpy@@GLIBC_2.14 (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008FE: main (gdbmem.cpp:47)
==10668==  Address 0x5204050 is 16 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
```

main函数47行，memcpy(ptmp, ptmp2, 20);写入已经释放的内存

```
==10668== 
==10668== Invalid read of size 1
==10668==    at 0x4ED0760: strchrnul (strchr.S:24)
==10668==    by 0x4E87207: __find_specmb (printf-parse.h:108)
==10668==    by 0x4E87207: vfprintf (vfprintf.c:1312)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x40090F: main (gdbmem.cpp:48)
==10668==  Address 0x5204040 is 0 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
==10668== 
==10668== Invalid read of size 1
==10668==    at 0x4E8741A: vfprintf (vfprintf.c:1324)
==10668==    by 0x4E8F898: printf (printf.c:33)
==10668==    by 0x40090F: main (gdbmem.cpp:48)
==10668==  Address 0x5204040 is 0 bytes inside a block of size 33 free'd
==10668==    at 0x4C2EDEB: free (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x4008D0: main (gdbmem.cpp:44)
==10668==  Block was alloc'd at
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400740: stringcopy(char*) (gdbmem.cpp:12)
==10668==    by 0x40087A: main (gdbmem.cpp:36)
```

main函数47行，printf(ptmp);写入已经释放的内存

```
0x40097d==10668== 
==10668== FILE DESCRIPTORS: 3 open at exit.
==10668== Open file descriptor 2: /dev/pts/4
==10668==    <inherited from parent>
==10668== 
==10668== Open file descriptor 1: /dev/pts/4
==10668==    <inherited from parent>
==10668== 
==10668== Open file descriptor 0: /dev/pts/4
==10668==    <inherited from parent>
```

Linux为了实现一切皆文件的设计哲学，不仅将数据抽象成了文件，也将一切操作和资源抽象成了文件，比如说硬件设备，socket，磁盘，进程，线程等。这样的设计将系统的所有动作都统一起来，实现了对系统的原子化操作，大大降低了维护和操作的难度。

```
==10668== 
==10668== 
==10668== HEAP SUMMARY:
==10668==     in use at exit: 100 bytes in 1 blocks
==10668==   total heap usage: 3 allocs, 2 frees, 1,157 bytes allocated
==10668== 
==10668== 100 bytes in 1 blocks are definitely lost in loss record 1 of 1
==10668==    at 0x4C2DB8F: malloc (in /usr/lib/valgrind/vgpreload_memcheck-amd64-linux.so)
==10668==    by 0x400888: main (gdbmem.cpp:38)
==10668== 
```

内存泄漏概述，3次内存分配，两次释放。已经有100个字节的内存已经确定泄漏。泄漏的内存分配于38行，char *ptmp2 = (char*)malloc(100);至此，内存泄漏检测完毕。


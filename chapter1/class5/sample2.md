## GDB实例分析2

The given code computes the factorial of a number erroneously. The goal of the debugging session is to pinpoint the reason of the error.

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

### Into the Debugger

Now follow the commands and the outputs carefully, especially the watchpoints. What I'm doing is basically:

- Setting a breakpoint just in the line of the function call
- Stepping into the function from that line
- Setting watchpoints for both the result of the calculation and the input number as it changes.
- Finally, analyzing the results from the watchpoints to find problematic behaviour

```
1.      $ g++ main.cpp -g -Wall -o main
2.      $ gdb main
3.      GNU gdb (GDB) Fedora (7.3-41.fc15)
4.      Copyright (C) 2011 Free Software Foundation, Inc.
5.      This GDB was configured as "i686-redhat-linux-gnu".
6.      For bug reporting instructions, please see:
7.      <http://www.gnu.org/software/gdb/bugs/>...
8.      Reading symbols from /home/manasij7479/Documents/main...done.
9.      (gdb) break 11
10.     Breakpoint 1 at 0x80485f9: file main.cpp, line 11.
11.     (gdb) run
12.     Starting program: /home/manasij7479/Documents/main
13.     3
14.     
15.     Breakpoint 1, main () at main.cpp:11
16.     11        long val=factorial(n);
17.     (gdb) step
18.     factorial (n=3) at main.cpp:19
19.     19        long result(1);
20.     (gdb) list
21.     14        return 0;
22.     15      }
23.     16
24.     17      long factorial(int n)
25.     18      {
26.     19        long result(1);
27.     20        while(n--)
28.     21        {
29.     22          result*=n;
30.     23        }
31.     (gdb) watch n
32.     Hardware watchpoint 2: n
33.     (gdb) watch result
34.     Hardware watchpoint 3: result
35.     (gdb) continue
36.     Continuing.
37.     Hardware watchpoint 3: result
38.     
39.     Old value = 0
40.     New value = 1
```

Notice that result starts from 0 and is initialized to 1.

```
41.     factorial (n=3) at main.cpp:20
42.     20        while(n--)
43.     (gdb)
```

Notice that I didn't put in a command, I just hit <return>. It re-executes the last command.

```
44.     Continuing.
45.     Hardware watchpoint 2: n
46.     
47.     Old value = 3
48.     New value = 2
```

Notice that n gets is immediately decremented from 3 to 2.

```
49.     0x08048654 in factorial (n=2) at main.cpp:20
50.     20        while(n--)
51.     (gdb)
52.     Continuing.
53.     Hardware watchpoint 3: result
54.     
55.     Old value = 1
56.     New value = 2
```

Now result becomes 2 (by multiplying result's earlier value with n's value). We've found the first bug! result is supposed to be evaluated by multiplying 3 * 2 * 1 but here the multiplication starts from 2. To correct it, we have to change the loop a bit, but before that, lets see if the rest of the calculation is correct.

```
57.     factorial (n=2) at main.cpp:20
58.     20        while(n--)
59.     (gdb)
60.     Continuing.
61.     Hardware watchpoint 2: n
62.     
63.     Old value = 2
64.     New value = 1
```

n gets decremented from 2 to 1. Result doesn't change since n is 1.

```
65.     0x08048654 in factorial (n=1) at main.cpp:20
66.     20        while(n--)
67.     (gdb)
68.     Continuing.
69.     Hardware watchpoint 2: n
70.     
71.     Old value = 1
72.     New value = 0
```

n gets decremented from 1 to 0.

```
73.     0x08048654 in factorial (n=0) at main.cpp:20
74.     20        while(n--)
75.     (gdb)
76.     Continuing.
77.     Hardware watchpoint 3: result
78.     
79.     Old value = 2
80.     New value = 0
```

Now result becomes 0 (by multiplying result's earlier value with n's value, 0). Another bug! How can result hold the value of the factorial when it is multiplied by 0? The loop must be stopped before n reaches 0.

```
81.     factorial (n=0) at main.cpp:20
82.     20        while(n--)
83.     (gdb)
84.     Continuing.
85.     Hardware watchpoint 2: n
86.     
87.     Old value = 0
88.     New value = -1
89.     0x08048654 in factorial (n=-1) at main.cpp:20
90.     20        while(n--)
91.     (gdb)
92.     Continuing.
```

Now n becomes -1 and the loop isn't permitted to run anymore because n-- returns 0, and the function returns result's current value 0. Let's see what happens when the function exits.

```
93.     
94.     Watchpoint 2 deleted because the program has left the block in
95.     which its expression is valid.
96.     
97.     Watchpoint 3 deleted because the program has left the block in
98.     which its expression is valid.
```

This is what happens to a watchpoint when the variable goes out of scope.

```
99.     0x08048605 in main () at main.cpp:11
100.    11        long val=factorial(n);
101.    (gdb) print val
102.    $1 = 1293357044
```

print val shows a garbage value because gdb points to a line before it is executed, not after.

```
103.    (gdb) next
104.    12        cout<<val;
105.    (gdb) continue
106.    Continuing.
107.    0[Inferior 1 (process 2499) exited normally]
108.    (gdb) quit
```

Here's what the fix should look like:

```
`while``(n>0) ``//doesn't let n reach 0``{``  ``result*=n;``  ``n--;        ``//decrements only after the evaluation``}`
```

## GDB in Conclusion

You have now seen enough to try GBD out on your own. Some important topics have not been touched upon here for the sake of simplicity, such as dealing with [segmentation faults and other kinds of crashes](https://www.cprogramming.com/debugging/segfaults.html) or using tools like [Valgrind](https://www.cprogramming.com/debugging/valgrind.html) to find memory leaks.

Remember that GDB comes built in with an excellent help system. Just type help in the (gdb) prompt and you will be presented with options of what you could need help with. For details about a specific command, use the syntax

```
help <command>
```

Another important point to note is the use of shortcuts (like 'q' for 'quit'). GDB lets you use shortcuts for commands when it is not ambigious.

After learning about GDB, you do not have to panic the next time your program goes crazy. You have an excellent weapon in your arsenal now.
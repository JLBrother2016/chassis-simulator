###gtest学习笔记
* [入门教程](https://github.com/google/googletest/blob/master/googletest/docs/Primer.md)
####gtest的特点
1. Google C ++测试框架不会在第一次测试失败时停止。相反，它只会停止当前的测试并继续下一个测试
2. Google C ++测试框架基于流行的xUnit架构，因此如果您之前使用过JUnit或PyUnit，那么您会感到宾至如归 
3. 独立--每个测试用例应该是独立的, 该用例的测试结果不受其他测试影响
4. 有效的组织架构，清晰的命名，各个测试用例针对不同的测试对象，而对单个测试对象而言，又可能多个测试用例对应该对象的多个功能。将用例以层次结构的形式组织起来，并使用清晰的命名，使得我们通过阅读用例名称即可明了该用例的功能。
5. 可移植、可复用。
6. 当用例失败时，提供尽可能多的有效信息。信息越清晰和全面，越方便定位问题、高效地找出程序中的bug
#####什么是单元测试？
* 单元测试一般由编码人员自己完成，它的目的是隔离程序部件，并证明这些单个部件满足预期的功能
* 以库的方式使用Google Test, 可以不提供Main函数--仅仅写单元测试的代码就足够了
* 以静态库方式使用Google Test链接上gtest.lib和gtest_main.lib，后面只需要写相关的单元测试代码了。因为gtest_main.lib库里面已经提供了一个默认的main函数初始化Google Test实现 。 如下： 
```
GTEST_API_ int main(int argc, char **argv) {
  printf("Running main() from gtest_main.ccn");
  //函数解析命令行，以便Google Test使用不同的选项，这允许通过命令行来控制单元测试程序的一些行为
  testing::InitGoogleTest(&argc, argv);   
  RUN_ALL_TESTS();
  return 0;
}
```

* 最后写好的程序用命令`g++ *.cpp -o test -lgtest -pthread`来进行编译
```
[==========] Running 2 tests from 1 test case.
[----------] Global test environment set-up.
[----------] 2 tests from Add
[ RUN      ] Add.PositiveNumber
[       OK ] Add.PositiveNumber (0 ms)
[ RUN      ] Add.NegativeNumber
[       OK ] Add.NegativeNumber (0 ms)
[----------] 2 tests from Add (1 ms total)

[----------] Global test environment tear-down
[==========] 2 tests from 1 test case ran. (1 ms total)
[  PASSED  ] 2 tests.
```
* 使用TEST()宏来定义一个测试函数
```
TEST(test_case_name, test_name) {   //注意TEST宏第一个参数是测试用例名，第二个参数是测试名。这个宏函数没有返回值。我们在宏函数体里面可以写一些测试代码。
 ... test body ...
//test_case_name和test_name必须是合法的c++标识符，并且不包含下划线_
}
```
####断言
* Google Test单元测试库里面最重要就是断言，断言是通过一系列断言宏实现的。断言宏就好像一个函数，它检查一个语句的条件是true还是false。当一个断言失败了，Google Test就会打印断言所在的源代码文件名，行号还有一些断言信息
* ASSERT_*这类断言宏失败时会生成致命失败的信息，然后终止当前的测试函数。EXPECT_*这类断言宏失败时会生成非致命失败的信息，不会终止当前的测试函数。通常EXPECT_*是更好的选择，因为它允许我们在一个测试中报告多个失败的测试结果。如果是一个非常严重的错误，使用ASSERT_*也是很有理由的。通常ASSERT_*失败了就会从测试函数中立即返回，可能会跳过测试函数后面的清理代码，这会导致一些资源泄露。
* 一个TEST宏就是生成一个测试，一个测试里面可以包含多个断言。假如一个测试用例对应一个被测试的函数，那么一个测试就是相当于更小一层次的测试情况，比如测试参数、测试返回值、测试各种异常情况。
* 一个TEST宏就是生成一个测试，一个测试里面可以包含多个断言。假如一个测试用例对应一个被测试的函数，那么一个测试就是相当于更小一层次的测试情况，比如测试参数、测试返回值、测试各种异常情况。
* 测试夹具（Test Fixture）
如果你发现多个测试处理相同的数据，你就可以使用测试夹具。测试夹具可以让多个不同的测试使用相同的环境。比如你需要测试一个Queue类，许多个测试都要操作多个Queue对象。你可以定义一个测试夹具来准备一下相同的数据，稍后TEST_F定义的测试就可以访问和复用这些数据。
```
TEST_F(test_case_name, test_name) {
 ... test body ...
}
```
* TEST_F的第一个参数test_case_name必须是测试夹具的类名。在使用TEST_F之前，必须定义好测试夹具。
    - 运行时每次都创建一个新的夹具。
    - 直接通过SetUp初始化。
    - 执行测试。
    - 调用TearDown清理。
    - 删除测试夹具。



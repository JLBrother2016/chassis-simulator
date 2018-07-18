###github学习笔记
####配置Git
* 首先在本地创建ssh key, 命令`ssh-keygen -t rsa -C "your_email@youremail.com"`
* 使用默认的一路回车就行。成功的话会在~/下生成.ssh文件夹，进去，打开id_rsa.pub，复制里面的key。
* 还需要设置username和email，因为github每次commit都会记录他们。
```
$ git config --global user.name "your name"                 //把自己的用户名配置为全局
$ git config --global user.email "your_email@youremail.com" //把邮箱配置为全局
```
进入要上传的仓库，右键git bash，添加远程地址：
`$ git remote add origin git@github.com:yourName/yourRepo.git`
* 执行 git init 以创建新的 git 仓库
* 本地仓库由 git 维护的三棵"树"组成。
    * 第一个是你的工作目录，它持有实际文件；
    * 第二个是暂存区（Index），它像个缓存区域，临时保存你的改动；(git commit)
    * 最后是 HEAD，它指向你最后一次提交的结果。
* 你可以提出更改（把它们添加到暂存区），使用如下命令：
```
    git add <filename>      //添加指定文件                  //.git是一个目录
    git add *               //添加所有文件
```
* 如下命令以实际提交改动：  
    `git commit -m "代码提交信息"` ---将改动提交到HEAD, 但没有提交到远程端
* 执行如下命令以将这些改动提交到远端仓库：
    `git push origin master` ---可以把 master 换成你想要推送的任何分支。 
* `git remote add origin <server>` ---如此你就能够将你的改动推送到所添加的服务器上去了。 
####分支
* 分支是用来将特性开发绝缘开来的。在你创建仓库的时候，master 是"默认的"分支。在其他分支上进行开发，完成后再将它们合并到主分支上。
* 创建一个叫做"feature_x"的分支，并切换过去：`git checkout -b feature_x`
* 切换回主分支：`git checkout master`
* 再把新建的分支删掉：`git branch -d feature_x`
* 除非你将分支推送到远端仓库，不然该分支就是 不为他人所见的：`git push origin <branch>`
####更新与合并
* 要更新你的本地仓库至最新改动，执行：`git pull`---从远程库中拉,就是从远程仓库中更新
* 要合并其他分支到你的当前分支（例如 master），执行：`git merge <branch>`----git 都会尝试去自动合并改动。
* 在合并改动之前，你可以使用如下命令预览差异：`git diff <source_branch> <target_branch>`
* `git checkout -- <filename>`--此命令会使用 HEAD 中的最新内容替换掉你的工作目录中的文件。已添加到暂存区的改动以及新文件都不会受到影响。(git的三个目录:当前工作目录, 暂存区, HEAD目录)
* 假如你想丢弃你在本地的所有改动与提交，可以到服务器上获取最新的版本历史，并将你本地主分支指向它：
    ```
    git fetch origin
    git reset --hard origin/master
    ```
####使用小贴士
* 内建的图形化 git：`gitk`
* 彩色的 git 输出：`git config color.ui true`
* 显示历史记录时，每个提交的信息只显示一行：`git config format.pretty oneline`
* 交互式添加文件到暂存区：`git add -i`
# How to show or change your Git username or email address?

$	git config user.name

or

$	git config --list

Finally, you can also see your Git username in the Git configuration file in your HOME directory on Unix systems, i.e., this file:

~/.gitconfig
My file on this new test system looks like this:

[user]
        name = Alvin J. Alexander
        email = [omitted]
[merge]
        tool = vimdiff


# How to change your Git username
You can change your Git username like this:

git config --global user.name "Alvin J. Alexander"
You can probably also just edit the Git config file in your HOME directory and change it there:

$	vi ~/.gitconfig

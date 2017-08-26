#!/bin/sh

git filter-branch --env-filter '
OLD_EMAIL="root@shikherverma.com"
CORRECT_NAME="Suryansh Agarwal"
CORRECT_EMAIL="suryansh.a98@gmail.com"
if [ "$GIT_COMMITTER_EMAIL" = "root@shikherverma.com" ]
then
    export GIT_COMMITTER_NAME="Suryansh Agarwal"
    export GIT_COMMITTER_EMAIL="suryansh.a98@gmail.com"
fi
if [ "$GIT_AUTHOR_EMAIL" = "root@shikherverma.com" ]
then
    export GIT_AUTHOR_NAME="Suryansh Agarwal"
    export GIT_AUTHOR_EMAIL="suryansh.a98@gmail.com"
fi
' --tag-name-filter cat -- --branches --tags

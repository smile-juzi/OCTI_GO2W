__conda_setup="$('/home/dog/Downloads/enter/bin/conda' 'shell.bash' 'hook' 2> /dev/null)"
if [ $? -eq 0 ]; then
    eval "$__conda_setup"
else
    if [ -f "/home/dog/Downloads/enter/etc/profile.d/conda.sh" ]; then
        . "/home/dog/Downloads/enter/etc/profile.d/conda.sh"
    else
        export PATH="/home/dog/Downloads/enter/bin:$PATH"
    fi
fi
unset __conda_setup

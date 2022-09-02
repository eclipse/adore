export ZSH="$HOME/.oh-my-zsh"

ZSH_THEME="robbyrussell"
PS1='%n -> %d $(git_prompt_info)> '
plugins=(git)
source $ZSH/oh-my-zsh.sh

export ZSH="$HOME/.oh-my-zsh"

ZSH_THEME="robbyrussell"

plugins=(git)
source $ZSH/oh-my-zsh.sh
autoload -U colors && colors
_git_repo_name() { 
    gittopdir=$(git rev-parse --git-dir 2> /dev/null)
    if [[ "foo$gittopdir" == "foo.git" ]]; then
        echo `basename $(pwd)`
    elif [[ "foo$gittopdir" != "foo" ]]; then
        echo `dirname $gittopdir | xargs basename`
    fi
}
_git_branch_name() {    
    git branch 2>/dev/null | awk '/^\*/ { print $2 }'
}    
 _git_is_dirty() { 
   git diff --quiet 2> /dev/null || echo '*'
 }

setopt prompt_subst


PROMPT='%B%F{grey}[%F{cyan}ADORe CLI %F %F{magenta}%~%F{grey} %(?.%F{green}.%F{red})=>(%?) %b%f] $ '
RPROMPT=$(git_prompt_info)


alias help='bash ${ADORE_SOURCE_DIR}/tools/adore-cli_help.sh'



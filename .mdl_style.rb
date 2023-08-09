# Style file for markdownlint

all

exclude_rule 'first-line-h1' # First line in file should be a top level header

exclude_rule 'ul-style'

exclude_rule 'no-multiple-blanks'

exclude_rule 'header-style'

# Line lenght
rule 'MD013', :line_length => 120, :ignore_code_blocks => true, :tables => false

# Unordered list indentation
rule 'MD007', :indent => 2

# Ordered list item prefix
rule 'MD029', :style => 'ordered'

# Multiple headers with the same content
rule 'no-duplicate-header', :allow_different_nesting => true

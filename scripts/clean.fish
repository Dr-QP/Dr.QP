#!/usr/bin/env fish

function clean
  set pattern $argv[1]
  set files_to_clean (eval "echo $pattern") # This emits the warning. Yet to figure how to do globbing in function
  if count $files_to_clean >/dev/null
      echo "Cleaning $pattern"
      rm -rf $files_to_clean
  end
end

# clean "**/build/*"
# clean "**/_build/*"
# clean "**/install/*"
# clean "**/log/*"
# clean "**/lcov/*"
# clean "**/.cache/*"
# clean "**/__pycache__/"
# clean "**/*.egg-info/"


set files_to_clean **/build/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/_build/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/install/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/log/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/lcov/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/.cache/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/__pycache__/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/.pytest_cache/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

set files_to_clean **/*.egg-info/*
if count $files_to_clean >/dev/null
    rm -rf $files_to_clean
end

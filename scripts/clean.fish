#!/usr/bin/env fish

function clean
    set pattern $argv[1]
    set files_to_clean (find . -path "$pattern" -print0 | string split0)

    if test (count $files_to_clean) -gt 0
        echo "Cleaning $pattern"
        rm -rf -- $files_to_clean
    end
end

clean "**/build/*"
clean "**/_build/*"
clean "**/install/*"
clean "**/log/*"
clean "**/lcov/*"
clean "**/.cache/*"
clean "**/__pycache__/*"
clean "**/.pytest_cache/*"
clean "**/*.coverage.*"
clean "**/*.egg-info/*"
clean "**/.tmp"

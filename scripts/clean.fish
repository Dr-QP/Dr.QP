#!/usr/bin/env fish

function clean
  set files_to_clean $argv[1]
  echo "Cleaning '$files_to_clean'..."
  rm -rf "$files_to_clean"
  echo "Done."
end

clean '**/build/*'
clean '**/_build/*'
clean '**/install/*'
clean '**/log/*'
clean '**/lcov/*'
clean '**/.cache/*'
clean '**/__pycache__/*'
clean '**/.pytest_cache/*'
clean '**/*.egg-info/*'
clean '**/.tmp'



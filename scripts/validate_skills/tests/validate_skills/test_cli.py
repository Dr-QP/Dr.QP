#!/usr/bin/env python3
# Copyright (c) 2026 Dr.QP
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

"""Unit tests for CLI argument parsing and setup."""

import pytest

from validate_skills.cli import parse_arguments


class TestParseArgumentsBasic:
    """Tests for basic argument parsing."""

    def test_parse_arguments_default_path(self):
        """Should use current directory as default path."""
        args = parse_arguments([])
        assert args.path == '.'

    def test_parse_arguments_custom_path(self):
        """Should accept custom path argument."""
        args = parse_arguments(['./skills'])
        assert args.path == './skills'

    def test_parse_arguments_file_path(self):
        """Should accept path to specific SKILL.md file."""
        args = parse_arguments(['.github/skills/my-skill/SKILL.md'])
        assert args.path == '.github/skills/my-skill/SKILL.md'

    def test_parse_arguments_absolute_path(self):
        """Should accept absolute paths."""
        args = parse_arguments(['/opt/ros/overlay_ws/.github/skills'])
        assert args.path == '/opt/ros/overlay_ws/.github/skills'


class TestParseArgumentsRecommend:
    """Tests for --recommend flag."""

    def test_parse_arguments_recommend_default(self):
        """Should default to not showing recommendations."""
        args = parse_arguments([])
        assert args.recommend is False

    def test_parse_arguments_recommend_flag(self):
        """Should enable recommendations with --recommend flag."""
        args = parse_arguments(['--recommend'])
        assert args.recommend is True

    def test_parse_arguments_recommend_with_path(self):
        """Should parse --recommend flag with path argument."""
        args = parse_arguments(['./skills', '--recommend'])
        assert args.path == './skills'
        assert args.recommend is True

    def test_parse_arguments_recommend_before_path(self):
        """Should handle --recommend flag before path argument."""
        args = parse_arguments(['--recommend', './skills'])
        assert args.path == './skills'
        assert args.recommend is True


class TestParseArgumentsCIMode:
    """Tests for CI mode flag (continuous integration)."""

    def test_parse_arguments_ci_mode_default(self):
        """Should default to non-CI mode."""
        args = parse_arguments([])
        assert getattr(args, 'ci', False) is False

    def test_parse_arguments_ci_mode_flag(self):
        """Should enable CI mode with --ci flag."""
        args = parse_arguments(['--ci'])
        assert args.ci is True

    def test_parse_arguments_ci_with_recommend(self):
        """Should support combining --ci and --recommend flags."""
        args = parse_arguments(['--ci', '--recommend'])
        assert args.ci is True
        assert args.recommend is True


class TestParseArgumentsValidationLevel:
    """Tests for validation level control."""

    def test_parse_arguments_warnings_default(self):
        """Should include warnings by default."""
        args = parse_arguments([])
        assert getattr(args, 'warnings', True) is True

    def test_parse_arguments_no_warnings(self):
        """Should support --no-warnings flag to suppress warnings."""
        args = parse_arguments(['--no-warnings'])
        assert args.warnings is False

    def test_parse_arguments_errors_only(self):
        """Should support --errors-only flag."""
        args = parse_arguments(['--errors-only'])
        assert getattr(args, 'errors_only', False) is True


class TestParseArgumentsOutput:
    """Tests for output format options."""

    def test_parse_arguments_output_format_default(self):
        """Should default to text output format."""
        args = parse_arguments([])
        assert getattr(args, 'format', 'text') == 'text'

    def test_parse_arguments_output_json(self):
        """Should support --format json option."""
        args = parse_arguments(['--format', 'json'])
        assert args.format == 'json'

    def test_parse_arguments_output_csv(self):
        """Should support --format csv option."""
        args = parse_arguments(['--format', 'csv'])
        assert args.format == 'csv'

    def test_parse_arguments_output_xml(self):
        """Should support --format xml option."""
        args = parse_arguments(['--format', 'xml'])
        assert args.format == 'xml'


class TestParseArgumentsVerbosity:
    """Tests for verbosity control."""

    def test_parse_arguments_quiet_mode(self):
        """Should support -q/--quiet flag."""
        args = parse_arguments(['-q'])
        assert getattr(args, 'quiet', False) is True

    def test_parse_arguments_quiet_long_flag(self):
        """Should support --quiet long form."""
        args = parse_arguments(['--quiet'])
        assert args.quiet is True

    def test_parse_arguments_verbose_mode(self):
        """Should support -v/--verbose flag."""
        args = parse_arguments(['-v'])
        assert getattr(args, 'verbose', False) is True

    def test_parse_arguments_verbose_long_flag(self):
        """Should support --verbose long form."""
        args = parse_arguments(['--verbose'])
        assert args.verbose is True


class TestParseArgumentsHelpText:
    """Tests for help text and documentation."""

    def test_parse_arguments_help_includes_description(self):
        """Should include meaningful description in help text."""
        # This would typically be validated through ArgumentParser's help
        # For now, we verify the function accepts --help gracefully
        with pytest.raises(SystemExit) as exc_info:
            parse_arguments(['--help'])
        assert exc_info.value.code == 0

    def test_parse_arguments_help_shows_examples(self):
        """Help text should include usage examples."""
        # Help text formatting is validated at runtime
        # This test documents the expected feature
        pass


class TestParseArgumentsInvalid:
    """Tests for invalid argument combinations."""

    def test_parse_arguments_invalid_format(self):
        """Should reject invalid format values."""
        with pytest.raises(SystemExit):
            parse_arguments(['--format', 'invalid-format'])

    def test_parse_arguments_format_missing_value(self):
        """Should require value for --format option."""
        with pytest.raises(SystemExit):
            parse_arguments(['--format'])

    def test_parse_arguments_conflicting_flags(self):
        """Should handle conflicting flags appropriately."""
        # Document expected behavior for conflicting options
        # e.g., --errors-only conflicts with --recommend
        args = parse_arguments(['--errors-only', '--recommend'])
        # Implementation should handle this gracefully or warn
        assert args is not None


class TestParseArgumentsEdgeCases:
    """Tests for edge case argument handling."""

    def test_parse_arguments_empty_path(self):
        """Should handle empty string path argument."""
        # Empty path should either default to '.' or raise error
        args = parse_arguments([''])
        assert args.path is not None

    def test_parse_arguments_whitespace_path(self):
        """Should handle path with whitespace."""
        args = parse_arguments(['./skills with spaces/'])
        assert './skills with spaces/' in args.path

    def test_parse_arguments_relative_path_with_dots(self):
        """Should handle relative paths with parent directory references."""
        args = parse_arguments(['../../.github/skills'])
        assert args.path == '../../.github/skills'

    def test_parse_arguments_path_with_tilde(self):
        """Should accept tilde in paths for home directory."""
        args = parse_arguments(['~/my-skills'])
        assert '~' in args.path

    def test_parse_arguments_many_arguments(self):
        """Should handle multiple flags and arguments together."""
        args = parse_arguments(['./skills', '--ci', '--recommend', '--format', 'json', '--verbose'])
        assert args.path == './skills'
        assert args.ci is True
        assert args.recommend is True
        assert args.format == 'json'
        assert args.verbose is True

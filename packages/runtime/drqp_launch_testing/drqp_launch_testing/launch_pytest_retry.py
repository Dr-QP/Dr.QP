# Copyright (c) 2017-2026 Anton Matosov
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

"""
pytest plugin making ``launch_pytest`` tests safe to retry with ``pytest-retry``.

``launch_pytest``'s ``pytest_pyfunc_call`` hookwrapper mutates ``pyfuncitem.obj``
in place every time it runs, dispatching on whatever ``pyfuncitem.obj`` currently
is. On the first run that is the original test function; on any later run (e.g.
``pytest-retry`` retrying the ``call`` phase after tearing down and re-creating
fixtures) it is the *previous attempt's wrapper*, closed over the now-closed
``event_loop``/``launch_service`` fixture instances. Re-wrapping that stale
wrapper mixes two event loop instances and crashes with ``RuntimeError: Event
loop is closed`` (or "is already running"), instead of retrying.

The fix: cache the pristine original test callable on the item the first time
it is seen, and always dispatch/re-wrap from that cached original -- never from
``pyfuncitem.obj``. Every invocation then rebinds cleanly to whatever
``event_loop``/``launch_service`` fixture values are current for that attempt.

Enable this for a test package by adding to its ``test/conftest.py``::

    pytest_plugins = ['drqp_launch_testing.launch_pytest_retry']

See ``drqp_launch_testing/test/shutdown_behavior/SPEC.md`` (combo 6) for the
executable proof, and ``.github/instructions/launch-testing.instructions.md``
for the project-level usage rules for ``@pytest.mark.flaky``.
"""

import functools
import inspect

import launch_pytest.plugin as _lp_plugin
import pytest


def _patched_pytest_pyfunc_call(pyfuncitem):
    """Retry-safe replacement for ``launch_pytest.plugin.pytest_pyfunc_call``."""
    if not _lp_plugin.is_launch_test(pyfuncitem):
        yield
        return

    # Use the pristine original function, cached on first sight -- not
    # pyfuncitem.obj, which may already be a previous attempt's wrapper.
    func = getattr(pyfuncitem, '_launch_pytest_original_obj', None)
    if func is None:
        func = pyfuncitem.obj
        pyfuncitem._launch_pytest_original_obj = func

    if _lp_plugin.has_shutdown_kwarg(pyfuncitem) and _lp_plugin.need_shutdown_test_item(func):
        error_msg = (
            'generator or async generator based launch test items cannot be marked with'
            ' shutdown=True'
        )
        pytest.fail(error_msg)
        yield
        return

    shutdown_test = _lp_plugin.is_shutdown_test(pyfuncitem)
    fixture = _lp_plugin.get_launch_test_fixture(pyfuncitem)
    scope = _lp_plugin.get_launch_test_fixture_scope(fixture)
    event_loop = pyfuncitem.funcargs['event_loop']
    ls = pyfuncitem.funcargs['launch_service']
    auto_shutdown = fixture._launch_pytest_fixture_options['auto_shutdown']
    on_shutdown = functools.partial(
        _lp_plugin.finalize_launch_service,
        ls,
        eprefix=f'When running test {func.__name__}',
        auto_shutdown=auto_shutdown,
    )
    before_test = on_shutdown if shutdown_test else None
    if inspect.iscoroutinefunction(func):
        pyfuncitem.obj = _lp_plugin.wrap_coroutine(func, event_loop, before_test)
    elif inspect.isgeneratorfunction(func):
        if scope != 'function':
            shutdown_item = pyfuncitem._launch_pytest_shutdown_item
            pyfuncitem.obj, shutdown_item.obj = _lp_plugin.wrap_generator(
                func, event_loop, on_shutdown
            )
            shutdown_item._fixtureinfo = shutdown_item.session._fixturemanager.getfixtureinfo(
                shutdown_item, shutdown_item.obj, shutdown_item.cls, funcargs=True
            )
        else:
            pyfuncitem.obj = _lp_plugin.wrap_generator_fscope(func, event_loop, on_shutdown)
    elif inspect.isasyncgenfunction(func):
        if scope != 'function':
            shutdown_item = pyfuncitem._launch_pytest_shutdown_item
            pyfuncitem.obj, shutdown_item.obj = _lp_plugin.wrap_asyncgen(
                func, event_loop, on_shutdown
            )
            shutdown_item._fixtureinfo = shutdown_item.session._fixturemanager.getfixtureinfo(
                shutdown_item, shutdown_item.obj, shutdown_item.cls, funcargs=True
            )
        else:
            pyfuncitem.obj = _lp_plugin.wrap_asyncgen_fscope(func, event_loop, on_shutdown)
    else:
        pyfuncitem.obj = _lp_plugin.wrap_func(func, event_loop, before_test)
    yield


def pytest_configure(config: pytest.Config) -> None:
    """Swap in the retry-safe ``pytest_pyfunc_call`` for ``launch_pytest``."""
    if getattr(_lp_plugin, '_drqp_launch_pytest_retry_patched', False):
        return
    pm = config.pluginmanager
    pm.unregister(_lp_plugin)
    _lp_plugin.pytest_pyfunc_call = pytest.hookimpl(hookwrapper=True, tryfirst=True)(
        _patched_pytest_pyfunc_call
    )
    _lp_plugin._drqp_launch_pytest_retry_patched = True
    pm.register(_lp_plugin, name='launch_pytest.plugin')

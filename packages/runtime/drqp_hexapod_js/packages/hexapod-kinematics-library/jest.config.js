/** @type {import('jest').Config} */

const config = {
  reporters: [
    'default',
  ],
  coverageReporters: [
    ['lcov', {"projectRoot": "../../../../../"}],
     'json'
  ],
  collectCoverage: true,
};

module.exports = config;

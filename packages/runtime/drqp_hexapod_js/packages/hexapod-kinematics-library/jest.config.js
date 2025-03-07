/** @type {import('jest').Config} */

const config = {
  reporters: [
    'default',
  ],
  coverageReporters: ['lcov', 'json'],
  collectCoverage: true,
};

module.exports = config;

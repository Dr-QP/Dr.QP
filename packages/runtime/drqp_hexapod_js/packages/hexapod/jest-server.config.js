/** @type {import('jest').Config} */

const config = {
  reporters: [
    'default',
  ],
  coverageReporters: ['lcov'],
  collectCoverage: true,
};

module.exports = config;

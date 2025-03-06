/** @type {import('jest').Config} */

const config = {
  reporters: [
    'default',
    ['jest-junit', { outputDirectory: 'jest_test_results', outputName: 'report-jest.xunit.xml' }],
    ['github-actions', { silent: false }],
  ],
  coverageReporters: ['lcov', 'json'],
  collectCoverage: true,
};

module.exports = config;

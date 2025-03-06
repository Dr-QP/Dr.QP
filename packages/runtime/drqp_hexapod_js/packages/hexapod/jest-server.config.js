/** @type {import('jest').Config} */

const config = {
  reporters: [
    'default',
    ['jest-junit', { outputDirectory: 'jest_test_results', outputName: 'server-report-jest.xunit.xml' }],
    ['github-actions', { silent: false }],
  ],
  coverageReporters: ['lcov'],
  collectCoverage: true,
};

module.exports = config;

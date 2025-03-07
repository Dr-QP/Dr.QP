// Importing the server causes it to bring in prettier via the rclnodejs package's idl_generator
// Keep it off for now
// Try upgrading to https://github.com/jestjs/jest/releases/tag/v30.0.0-alpha.7 or newer
// Tracked in https://github.com/jestjs/jest/issues/14305
// import server from "./index"

describe("server", () => {
    it("should work", () => {
        expect(true).toBe(true)
    })
})

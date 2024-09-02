// vue.config.js
module.exports = {
  configureWebpack: {
    resolve: {
      fallback: {
        "os": require.resolve("os-browserify/browser"),
        "path": require.resolve("path-browserify"),
        "util": require.resolve("util/")
      }
    }
  }
}

const path = require('path');

module.exports = function (context, options) {
  return {
    name: 'my-webpack-alias-plugin',
    configureWebpack(config, isServer) {
      return {
        resolve: {
          alias: {
            '@/components': path.resolve(context.siteDir, 'src/components'),
            '@/lib': path.resolve(context.siteDir, 'src/lib'),
            '@splinetool/react-spline': path.resolve(context.siteDir, 'node_modules/@splinetool/react-spline/dist/react-spline.js'),
          },
        },
      };
    },
  };
};
"use strict";
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || function (mod) {
    if (mod && mod.__esModule) return mod;
    var result = {};
    if (mod != null) for (var k in mod) if (k !== "default" && Object.prototype.hasOwnProperty.call(mod, k)) __createBinding(result, mod, k);
    __setModuleDefault(result, mod);
    return result;
};
var __importDefault = (this && this.__importDefault) || function (mod) {
    return (mod && mod.__esModule) ? mod : { "default": mod };
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.wrapServerCallback = exports.ServiceServer = void 0;
var react_1 = __importStar(require("react"));
var prop_types_1 = __importDefault(require("prop-types"));
var RosConnection_1 = require("../RosConnection");
var roslib_1 = require("roslib");
var ServiceServer = function (props) {
    var name = props.name, serviceType = props.serviceType, callback = props.callback;
    var ros = (0, RosConnection_1.useRos)();
    (0, react_1.useEffect)(function () {
        var service = new roslib_1.Service({ ros: ros, name: name, serviceType: serviceType });
        service.advertise(wrapServerCallback(callback));
        return function () {
            service.unadvertise();
        };
    }, []);
    return react_1.default.createElement(react_1.Fragment, null);
};
exports.ServiceServer = ServiceServer;
;
exports.ServiceServer.propTypes = {
    name: prop_types_1.default.string.isRequired,
    serviceType: prop_types_1.default.string.isRequired,
    callback: prop_types_1.default.func.isRequired,
};
function wrapServerCallback(callback) {
    return function (request, response) {
        callback(request, response);
        return true;
    };
}
exports.wrapServerCallback = wrapServerCallback;
//# sourceMappingURL=ServiceServer.js.map
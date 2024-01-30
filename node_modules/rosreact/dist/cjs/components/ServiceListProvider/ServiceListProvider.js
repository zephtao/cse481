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
exports.useServiceList = exports.getServiceList = exports.ServiceListProvider = void 0;
var react_1 = __importStar(require("react"));
var prop_types_1 = __importDefault(require("prop-types"));
var RosConnection_1 = require("../RosConnection");
var common_1 = require("../common");
var initialServices = [];
var ServiceListContext = (0, react_1.createContext)(initialServices);
var ServiceListProvider = function (props) {
    var ros = (0, RosConnection_1.useRos)();
    var _a = (0, react_1.useState)(initialServices), serviceList = _a[0], setServiceList = _a[1];
    var callback = function (newServiceList) {
        setServiceList(newServiceList);
    };
    (0, react_1.useEffect)(function () {
        if (props.trigger || props.trigger === undefined) {
            getServiceList(ros, callback, props.failedCallback);
        }
    }, [props.trigger]);
    return (react_1.default.createElement(ServiceListContext.Provider, { value: serviceList }, props.children));
};
exports.ServiceListProvider = ServiceListProvider;
exports.ServiceListProvider.propTypes = {
    children: prop_types_1.default.node,
    trigger: prop_types_1.default.bool,
    failedCallback: prop_types_1.default.func,
};
function getServiceList(ros, callback, failedCallback) {
    ros.getServices(callback, failedCallback);
}
exports.getServiceList = getServiceList;
function useServiceList() {
    return (0, common_1.useCheckedContext)(ServiceListContext);
}
exports.useServiceList = useServiceList;
//# sourceMappingURL=ServiceListProvider.js.map
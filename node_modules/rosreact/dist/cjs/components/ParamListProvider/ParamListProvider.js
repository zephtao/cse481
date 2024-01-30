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
exports.useParamList = exports.getParamList = exports.ParamListProvider = void 0;
var react_1 = __importStar(require("react"));
var prop_types_1 = __importDefault(require("prop-types"));
var common_1 = require("../common");
var RosConnection_1 = require("../RosConnection");
var initialParams = [];
var ParamListContext = (0, react_1.createContext)(initialParams);
var ParamListProvider = function (props) {
    var ros = (0, RosConnection_1.useRos)();
    var get = props.trigger;
    var _a = (0, react_1.useState)(initialParams), paramList = _a[0], setParamList = _a[1];
    var callback = function (newParamList) {
        setParamList(newParamList);
    };
    (0, react_1.useEffect)(function () {
        if (props.trigger || props.trigger === undefined) {
            getParamList(ros, callback, props.failedCallback);
        }
    }, [props.trigger]);
    return (react_1.default.createElement(ParamListContext.Provider, { value: paramList }, props.children));
};
exports.ParamListProvider = ParamListProvider;
exports.ParamListProvider.propTypes = {
    children: prop_types_1.default.node,
    trigger: prop_types_1.default.bool,
    failedCallback: prop_types_1.default.func,
};
function getParamList(ros, callback, failedCallback) {
    ros.getParams(callback, failedCallback);
}
exports.getParamList = getParamList;
function useParamList() {
    return (0, common_1.useCheckedContext)(ParamListContext);
}
exports.useParamList = useParamList;
//# sourceMappingURL=ParamListProvider.js.map
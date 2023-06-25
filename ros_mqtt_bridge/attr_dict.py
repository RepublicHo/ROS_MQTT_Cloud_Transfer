#!/usr/bin/env python
# coding: utf-8

class AttrDict(dict):
    """
    A custom dictionary class that allows accessing dictionary keys as object attributes.
    """

    def __getattr__(self, name):
        """
        Called when an attribute is accessed using dot notation (e.g., my_dict.key).
        Returns the value of the requested key if it exists in the dictionary.
        Raises an AttributeError if the key does not exist.
        """
        if name in self:
            return self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    def __setattr__(self, name, value):
        """
        Called when an attribute is set using dot notation (e.g., my_dict.key = value).
        Sets the specified key-value pair in the dictionary using the attribute name as the key
        and the attribute value as the value. If the attribute value is itself a dictionary or list,
        it recursively converts it to an AttrDict or list of AttrDict objects.
        """
        self[name] = AttrDict.set_recursively(value)

    def __delattr__(self, name):
        """
        Called when an attribute is deleted using dot notation (e.g., del my_dict.key).
        Deletes the specified key from the dictionary if it exists.
        Raises an AttributeError if the key does not exist.
        """
        if name in self:
            del self[name]
        else:
            raise AttributeError("No such attribute: " + name)

    @staticmethod
    def set_recursively(_obj, attr_dict=None):
        """
        A static method that is used internally by the __setattr__ method to recursively convert
        nested dictionaries and lists to AttrDict and list of AttrDict objects.
        """
        if attr_dict is None:
            attr_dict = AttrDict()

        # If _obj is a dictionary, recursively convert nested dictionaries and lists to AttrDict and list of AttrDict objects
        if isinstance(_obj, dict):
            for k, v in _obj.items():
                if isinstance(v, (dict, list)):
                    attr_dict[k] = AttrDict.set_recursively(v)
                else:
                    attr_dict[k] = v
            return attr_dict

        # If _obj is a list, recursively convert nested dictionaries and lists to AttrDict and list of AttrDict objects
        elif isinstance(_obj, list):
            _list = []
            for v in _obj:
                _list.append(AttrDict.set_recursively(v))
            return _list

        # If _obj is neither a dictionary nor a list, return it as is
        else:
            return _obj
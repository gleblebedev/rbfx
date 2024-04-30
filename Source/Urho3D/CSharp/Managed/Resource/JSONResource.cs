// Copyright (c) 2024-2024 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

using System;
using System.Text.Json;

namespace Urho3DNet
{
    public abstract class JSONResource<T>: JSONResourceBase
    {
        /// <summary>
        /// Stored resource value.
        /// </summary>
        private T value_;

        /// <summary>
        /// Json Serializer Options.
        /// </summary>
        private static readonly JsonSerializerOptions JsonSerializerOptions;

        public JSONResource(Context context): base(context)
        {
        }

        static JSONResource()
        {
            JsonSerializerOptions = new JsonSerializerOptions(JsonSerializerOptions.Default) { WriteIndented = true };
        }

        /// <summary>
        /// Stored resource value.
        /// </summary>
        public T Value { get => value_; set => value_ = value; }

        public override bool DeserializeJson(string jsonString)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(jsonString))
                    value_ = default;
                else
                    value_ = JsonSerializer.Deserialize<T>(jsonString);
                return true;
            }
            catch (Exception ex)
            {
                Log.Error(ex.Message);
                return false;
            }
        }

        public override string SerializeJson()
        {
            try
            {
                if (value_ == null)
                    return string.Empty;
                else
                    return JsonSerializer.Serialize(value_, JsonSerializerOptions.Default);
            }
            catch (Exception ex)
            {
                Log.Error(ex.Message);
                return string.Empty;
            }
        }
    }
}

// Copyright (c) 2024-2024 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Urho3DNet
{
    public abstract class JSONResource<T> : JSONResourceBase
    {
        /// <summary>
        /// Stored resource value.
        /// </summary>
        private T value_;

        /// <summary>
        /// Cached Json Serializer Options.
        /// </summary>
        private static JsonSerializerOptions jsonSerializerOptions_;

        public JSONResource(Context context) : base(context)
        {
            if (jsonSerializerOptions_ == null)
            {
                jsonSerializerOptions_ = new JsonSerializerOptions(JsonSerializerOptions.Default) { WriteIndented = true };
                jsonSerializerOptions_.Converters.Add(new IntVector2JsonConverter());
                jsonSerializerOptions_.Converters.Add(new IntVector3JsonConverter());
                jsonSerializerOptions_.Converters.Add(new Vector2JsonConverter());
                jsonSerializerOptions_.Converters.Add(new Vector3JsonConverter());
                jsonSerializerOptions_.Converters.Add(new Vector4JsonConverter());
                jsonSerializerOptions_.Converters.Add(new QuaternionJsonConverter());
                jsonSerializerOptions_.Converters.Add(new BoundingBoxJsonConverter());
                jsonSerializerOptions_.Converters.Add(new BoundingBoxJsonConverter());
                jsonSerializerOptions_.Converters.Add(new ResourceJsonConverter(context));
                jsonSerializerOptions_.Converters.Add(new JsonStringEnumConverter());
            }
        }

        /// <summary>
        /// Json Serializer Options.
        /// </summary>
        private static JsonSerializerOptions JsonSerializerOptions => jsonSerializerOptions_;

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
                    value_ = JsonSerializer.Deserialize<T>(jsonString, JsonSerializerOptions);
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
                    return JsonSerializer.Serialize(value_, JsonSerializerOptions);
            }
            catch (Exception ex)
            {
                Log.Error(ex.Message);
                return string.Empty;
            }
        }

        #region IntVector* Converters
        internal class IntVector2JsonConverter : JsonConverter<IntVector2>
        {
            public override IntVector2 Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                new SpaceSeparatedVauleHelper(reader.GetString()!).Parse(helper => new IntVector2(helper.ReadInt(), helper.ReadInt()));


            public override void Write(
                Utf8JsonWriter writer,
                IntVector2 value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(string.Format(CultureInfo.InvariantCulture, "{0} {1}", value.X, value.Y));
        }

        internal class IntVector3JsonConverter : JsonConverter<IntVector3>
        {
            public override IntVector3 Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                new SpaceSeparatedVauleHelper(reader.GetString()!).Parse(helper =>
                    new IntVector3(helper.ReadInt(), helper.ReadInt(), helper.ReadInt()));

            public override void Write(
                Utf8JsonWriter writer,
                IntVector3 value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(string.Format(CultureInfo.InvariantCulture, "{0} {1} {2}", value.X, value.Y, value.Z));
        }
        #endregion

        #region IntVector* Converters
        internal class Vector2JsonConverter : JsonConverter<Vector2>
        {
            public override Vector2 Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                new SpaceSeparatedVauleHelper(reader.GetString()!).Parse(helper => new Vector2(helper.ReadFloat(), helper.ReadFloat()));


            public override void Write(
                Utf8JsonWriter writer,
                Vector2 value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(string.Format(CultureInfo.InvariantCulture, "{0} {1}", value.X, value.Y));
        }

        internal class Vector3JsonConverter : JsonConverter<Vector3>
        {
            public override Vector3 Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                new SpaceSeparatedVauleHelper(reader.GetString()!).Parse(helper =>
                    new Vector3(helper.ReadFloat(), helper.ReadFloat(), helper.ReadFloat()));

            public override void Write(
                Utf8JsonWriter writer,
                Vector3 value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(string.Format(CultureInfo.InvariantCulture, "{0} {1} {2}", value.X, value.Y, value.Z));
        }

        internal class Vector4JsonConverter : JsonConverter<Vector4>
        {
            public override Vector4 Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                new SpaceSeparatedVauleHelper(reader.GetString()!).Parse(helper =>
                    new Vector4(helper.ReadFloat(), helper.ReadFloat(), helper.ReadFloat(), helper.ReadFloat()));

            public override void Write(
                Utf8JsonWriter writer,
                Vector4 value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(string.Format(CultureInfo.InvariantCulture, "{0} {1} {2} {3}", value.X, value.Y, value.Z, value.W));
        }
        #endregion

        internal class ResourceJsonConverter : JsonConverter<Resource>
        {
            private readonly Context _context;

            public override bool CanConvert(Type typeToConvert)
            {
                return typeof(Resource).IsAssignableFrom(typeToConvert);
            }

            public ResourceJsonConverter(Context context)
            {
                _context = context;
            }

            public override Resource Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options)
            {
                var str = reader.GetString()!;
                if (string.IsNullOrWhiteSpace(str))
                    return null;
                var split = str.IndexOf(';');
                if (split < 0)
                    return null;

                return _context.ResourceCache.GetResource(str.Substring(0, split), str.Substring(split + 1));
            }


            public override void Write(
                Utf8JsonWriter writer,
                Resource value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue((value == null) ? "" : string.Format(CultureInfo.InvariantCulture, "{0};{1}", value.GetTypeName(), value.Name));
        }

        internal class QuaternionJsonConverter : JsonConverter<Quaternion>
        {
            public override Quaternion Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                new SpaceSeparatedVauleHelper(reader.GetString()!).Parse(helper =>
                    new Quaternion(helper.ReadFloat(), helper.ReadFloat(), helper.ReadFloat(), helper.ReadFloat()));

            public override void Write(
                Utf8JsonWriter writer,
                Quaternion value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(string.Format(CultureInfo.InvariantCulture, "{0} {1} {2} {3}", value.W, value.X, value.Y, value.Z));
        }

        internal class BoundingBoxJsonConverter : JsonConverter<BoundingBox>
        {
            private static readonly Vector3JsonConverter _vector3JsonConverter = new Vector3JsonConverter();

            public override BoundingBox Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options)
            {
                if (reader.TokenType != JsonTokenType.StartObject)
                {
                    throw new JsonException();
                }

                BoundingBox bbox = new BoundingBox(0.0f, 0.0f);
                while (reader.Read())
                {
                    if (reader.TokenType == JsonTokenType.EndObject)
                    {
                        return bbox;
                    }

                    // Get the key.
                    if (reader.TokenType != JsonTokenType.PropertyName)
                    {
                        throw new JsonException();
                    }

                    string? propertyName = reader.GetString();

                    // Get the value.
                    reader.Read();

                    switch (propertyName)
                    {
                        case "min": bbox.Min = _vector3JsonConverter.Read(ref reader, typeof(Vector3), options); break;
                        case "max": bbox.Max = _vector3JsonConverter.Read(ref reader, typeof(Vector3), options); break;
                        default: throw new JsonException($"Unknown property \"{propertyName}\"."); 
                    }
                }

                throw new JsonException();
            }

            public override void Write(
                Utf8JsonWriter writer,
                BoundingBox value,
                JsonSerializerOptions options)

            {
                writer.WriteStartObject();
                writer.WritePropertyName("min");
                _vector3JsonConverter.Write(writer, value.Min, options);
                writer.WritePropertyName("max");
                _vector3JsonConverter.Write(writer, value.Max, options);
                writer.WriteEndObject();
            }
        }
        internal struct SpaceSeparatedVauleHelper
        {
            private int _position;
            private string _string;

            public SpaceSeparatedVauleHelper(string str)
            {
                _position = 0;
                _string = str;
            }

            internal int ReadInt()
            {
                while (_position < _string.Length && Char.IsWhiteSpace(_string[_position]))
                {
                    ++_position;
                }
                var start = _position;
                while (_position < _string.Length && !Char.IsWhiteSpace(_string[_position]) && _string[_position] != ' ')
                {
                    ++_position;
                }
                if (start != _position)
                {
                    if (int.TryParse(_string.Substring(start, _position - start), NumberStyles.Any,
                            CultureInfo.InvariantCulture, out var value))
                        return value;
                }

                return default;
            }
            internal float ReadFloat()
            {
                while (_position < _string.Length && Char.IsWhiteSpace(_string[_position]))
                {
                    ++_position;
                }
                var start = _position;
                while (_position < _string.Length && !Char.IsWhiteSpace(_string[_position]) && _string[_position] != ' ')
                {
                    ++_position;
                }
                if (start != _position)
                {
                    if (float.TryParse(_string.Substring(start, _position - start), NumberStyles.Any,
                            CultureInfo.InvariantCulture, out var value))
                        return value;
                }

                return default;
            }

            internal T Parse<T>(Func<SpaceSeparatedVauleHelper, T> func)
            {
                return func(this);
            }
        }
    }
}

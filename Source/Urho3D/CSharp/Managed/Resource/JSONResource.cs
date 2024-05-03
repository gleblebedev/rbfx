// Copyright (c) 2024-2024 the rbfx project.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

using System;
using System.Globalization;
using System.Text.Json;
using System.Text.Json.Serialization;

namespace Urho3DNet
{
    /// <summary>
    /// Base class for resource files. The file contains value serialized to json via .net serialization.
    /// </summary>
    /// <typeparam name="TValue">Resource value type.</typeparam>
    public abstract class JSONResource<TValue> : JSONResourceBase where TValue : new()
    {
        /// <summary>
        /// Stored resource value.
        /// </summary>
        private TValue value_;

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
                jsonSerializerOptions_.Converters.Add(new IntRectJsonConverter());
                jsonSerializerOptions_.Converters.Add(new Vector2JsonConverter());
                jsonSerializerOptions_.Converters.Add(new Vector3JsonConverter());
                jsonSerializerOptions_.Converters.Add(new Vector4JsonConverter());
                jsonSerializerOptions_.Converters.Add(new RectJsonConverter());
                jsonSerializerOptions_.Converters.Add(new QuaternionJsonConverter());
                jsonSerializerOptions_.Converters.Add(new BoundingBoxJsonConverter());
                jsonSerializerOptions_.Converters.Add(new BoundingBoxJsonConverter());
                jsonSerializerOptions_.Converters.Add(new ResourceJsonConverter(context));
                jsonSerializerOptions_.Converters.Add(new ResourceRefJsonConverter(context));
                jsonSerializerOptions_.Converters.Add(new ResourceRefListJsonConverter(context));
                jsonSerializerOptions_.Converters.Add(new JsonStringEnumConverter());
            }
        }

        /// <summary>
        /// Json Serializer Options.
        /// </summary>
        public static JsonSerializerOptions JsonSerializerOptions => jsonSerializerOptions_;

        /// <summary>
        /// Stored resource value.
        /// </summary>
        public TValue Value { get => value_; set => value_ = value; }

        /// <summary>
        /// Parses the text representing a single JSON value into a <typeparamref name="TValue"/>.
        /// </summary>
        /// <param name="jsonString">JSON text to parse.</param>
        public override bool DeserializeJson(string jsonString)
        {
            try
            {
                if (string.IsNullOrWhiteSpace(jsonString))
                    value_ = default;
                else
                    value_ = JsonSerializer.Deserialize<TValue>(jsonString, JsonSerializerOptions);
                return true;
            }
            catch (Exception ex)
            {
                Log.Error(ex.Message);
                return false;
            }
        }

        /// <summary>
        /// Converts the provided value into a <see cref="string"/>.
        /// </summary>
        /// <returns>A <see cref="string"/> representation of the value.</returns>
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
            public override IntVector2 Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => IntVector2.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, IntVector2 value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
        }

        internal class IntVector3JsonConverter : JsonConverter<IntVector3>
        {
            public override IntVector3 Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => IntVector3.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, IntVector3 value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
        }

        internal class IntRectJsonConverter : JsonConverter<IntRect>
        {
            public override IntRect Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => IntRect.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, IntRect value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
        }

        #endregion

        #region Vector* Converters
        internal class Vector2JsonConverter : JsonConverter<Vector2>
        {
            public override Vector2 Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => Vector2.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, Vector2 value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
        }

        internal class Vector3JsonConverter : JsonConverter<Vector3>
        {
            public override Vector3 Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => Vector3.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, Vector3 value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
        }

        internal class Vector4JsonConverter : JsonConverter<Vector4>
        {
            public override Vector4 Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => Vector4.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, Vector4 value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
        }

        internal class RectJsonConverter : JsonConverter<Rect>
        {
            public override Rect Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => Rect.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, Rect value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
        }
        #endregion

        #region Other Converters
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
            public override Quaternion Read(ref Utf8JsonReader reader, Type typeToConvert, JsonSerializerOptions options) => Quaternion.Parse(reader.GetString()!);

            public override void Write(Utf8JsonWriter writer, Quaternion value, JsonSerializerOptions options) => writer.WriteStringValue(value.ToString());
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

        internal class ResourceRefJsonConverter : JsonConverter<ResourceRef>
        {
            private readonly Context _context;

            public ResourceRefJsonConverter(Context context)
            {
                _context = context;
            }

            public override ResourceRef Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                ResourceRef.Parse(reader.GetString());

            public override void Write(
                Utf8JsonWriter writer,
                ResourceRef value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(value.ToString(_context));
        }

        internal class ResourceRefListJsonConverter : JsonConverter<ResourceRefList>
        {
            private readonly Context _context;

            public ResourceRefListJsonConverter(Context context)
            {
                _context = context;
            }

            public override ResourceRefList Read(
                ref Utf8JsonReader reader,
                Type typeToConvert,
                JsonSerializerOptions options) =>
                ResourceRefList.Parse(reader.GetString());

            public override void Write(
                Utf8JsonWriter writer,
                ResourceRefList value,
                JsonSerializerOptions options) =>
                writer.WriteStringValue(value.ToString(_context));
        }
        #endregion

    }
}

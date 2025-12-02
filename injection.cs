
using System.Data;
using System.Data.SqlClient; 
using System.Threading.Tasks;
private static async Task UpdateDemographicsAsync(int customerId, string demoXml, string connectionString)
{
    const string commandText =
        "UPDATE Sales.Store SET Demographics = @demographics WHERE CustomerID = @ID;";

    using (var connection = new SqlConnection(connectionString))
    using (var command = new SqlCommand(commandText, connection))
    {
        // explicit parameter types (preferred over AddWithValue)
        command.Parameters.Add("@ID", SqlDbType.Int).Value = customerId;
        command.Parameters.Add("@demographics", SqlDbType.Xml).Value = (object)demoXml ?? DBNull.Value;

        try
        {
            await connection.OpenAsync();
            int rowsAffected = await command.ExecuteNonQueryAsync();
            Console.WriteLine("RowsAffected: {0}", rowsAffected);
        }
        catch (Exception ex)
        {
            // do NOT log demoXml (sensitive), log safe details only
            Console.WriteLine($"Update failed for CustomerID={customerId}: {ex.Message}");
            throw;
        }
    }
}

// Requires: using System.Data; using System.Data.SqlClient; using System.Threading.Tasks;
public static async Task<UserDto?> GetUserByCredentialsAsync(string username, string password, string connectionString)
{
    // DO NOT compare plaintext in DB. This example fetches the user by username only.
    const string query = "SELECT UserId, Username, PasswordHash FROM Users WHERE Username = @Username;";

    using (var conn = new SqlConnection(connectionString))
    using (var cmd = new SqlCommand(query, conn))
    {
        cmd.Parameters.Add("@Username", SqlDbType.NVarChar, 256).Value = username;

        await conn.OpenAsync();
        using (var reader = await cmd.ExecuteReaderAsync(CommandBehavior.SingleRow))
        {
            if (!await reader.ReadAsync()) return null;

            return new UserDto
            {
                UserId = reader.GetInt32(0),
                Username = reader.GetString(1),
                PasswordHash = reader.IsDBNull(2) ? null : reader.GetString(2)
            };
        }
    }
}

public class UserDto
{
    public int UserId { get; set; }
    public string Username { get; set; } = "";
    public string? PasswordHash { get; set; }
}

// Requires:
using Microsoft.AspNetCore.Identity;
using Microsoft.EntityFrameworkCore;
using System.Threading.Tasks;

public class UserService
{
    private readonly DbContext _dbContext;

    public UserService(DbContext dbContext)
    {
        _dbContext = dbContext;
    }

    // Register user (hash password)
    public async Task RegisterAsync(string username, string plainPassword)
    {
        var exists = await _dbContext.Set<AppUser>().AnyAsync(u => u.Username == username);
        if (exists) throw new InvalidOperationException("User exists");

        var hasher = new PasswordHasher<AppUser>();
        var user = new AppUser { Username = username };
        user.PasswordHash = hasher.HashPassword(user, plainPassword);

        _dbContext.Add(user);
        await _dbContext.SaveChangesAsync();
    }

    // Login: fetch by username, verify hash
    public async Task<bool> VerifyLoginAsync(string username, string plainPassword)
    {
        var user = await _dbContext.Set<AppUser>().SingleOrDefaultAsync(u => u.Username == username);
        if (user == null) return false;

        var hasher = new PasswordHasher<AppUser>();
        var result = hasher.VerifyHashedPassword(user, user.PasswordHash, plainPassword);
        return result == PasswordVerificationResult.Success;
    }
}

public class AppUser
{
    public int Id { get; set; }
    public string Username { get; set; } = "";
    public string PasswordHash { get; set; } = "";
}

// Register user (hash password)
public async Task RegisterAsync(DbContext db, string username, string plainPassword)
{
    var exists = await db.Set<AppUser>().AnyAsync(u => u.Username == username);
    if (exists) throw new InvalidOperationException("User exists");

    var hasher = new PasswordHasher<AppUser>();
    var user = new AppUser { Username = username };
    user.PasswordHash = hasher.HashPassword(user, plainPassword);

    db.Add(user);
    await db.SaveChangesAsync();
}

// Login: fetch by username, verify hash
public async Task<bool> VerifyLoginAsync(DbContext db, string username, string plainPassword)
{
    var user = await db.Set<AppUser>().SingleOrDefaultAsync(u => u.Username == username);
    if (user == null) return false;

    var hasher = new PasswordHasher<AppUser>();
    var result = hasher.VerifyHashedPassword(user, user.PasswordHash, plainPassword);
    return result == PasswordVerificationResult.Success;
}

// Requires: Gremlin.Net.Client from NuGet
// using Gremlin.Net.Driver; using System.Collections.Generic; using System.Threading.Tasks;

public static async Task AddVertexSafeAsync(GremlinClient gremlinClient, string label, string nodeId)
{
    // Use bindings instead of string concatenation
    var gremlinQuery = "g.addV(label).property('id', nodeId)";

    var bindings = new Dictionary<string, object>
    {
        { "label", label },     // label should normally be validated/whitelisted
        { "nodeId", nodeId }
    };

    try
    {
        await gremlinClient.SubmitAsync<dynamic>(gremlinQuery, bindings);
    }
    catch (Exception ex)
    {
        Console.WriteLine($"Gremlin add vertex failed: {ex.Message}");
        throw;
    }
}

// Example safe existence check using bindings
public static async Task<string?> GetVertexIdIfExistsAsync(GremlinClient gremlinClient, string label, string nodeId)
{
    var query = "g.V().has(label, 'id', nodeId).values('id').tryNext().orElse(null)";
    var bindings = new Dictionary<string, object> { { "label", label }, { "nodeId", nodeId } };

    var result = await gremlinClient.SubmitWithSingleResultAsync<object?>(query, bindings);
    return result?.ToString();
}
